const $ = (id) => document.getElementById(id);
const sourceValues = { opencv: "0", aravis: "0", recording: "" };
const sourceNames = { opencv: "USB 相机", aravis: "GigE / GenICam", recording: "本地录像" };
let source = "opencv";
let processState = "idle";
let currentState = null;
let currentSequence = -1;
let selectedId = null;
let lastTargetIds = new Set();
let lastStatus = null;
let firstStatus = true;

function setMessage(value, success = false) {
  $("message").textContent = value || "";
  $("message").classList.toggle("success", success);
}

function addEvent(title, detail, kind = "normal") {
  const row = document.createElement("div");
  row.className = "event";
  const dot = document.createElement("span");
  dot.className = "event-dot " + kind;
  const copy = document.createElement("div");
  const heading = document.createElement("strong");
  heading.textContent = title;
  const small = document.createElement("small");
  small.textContent = `${new Date().toLocaleTimeString("zh-CN", { hour12: false })} · ${detail}`;
  copy.append(heading, small);
  row.append(dot, copy);
  $("events").prepend(row);
  while ($("events").children.length > 5) $("events").lastElementChild.remove();
}

function setSource(next) {
  sourceValues[source] = $("sourceValue").value;
  source = next;
  document.querySelectorAll(".source-tab").forEach((button) => {
    button.classList.toggle("active", button.dataset.source === next);
  });
  const recording = next === "recording";
  $("sourceLabel").textContent = recording ? "录像路径" : "设备编号";
  $("sourceValue").placeholder = recording ? "/path/to/camera-recording.mp4" : "0";
  $("sourceValue").value = sourceValues[next];
  $("browseButton").hidden = !recording;
  $("loopOption").hidden = !recording;
  $("sourceHint").textContent = recording
    ? "可输入本机文件路径，或选择文件上传后回放。"
    : next === "aravis" ? "设备编号对应 GenICam / Aravis 相机。" : "设备编号对应 OpenCV VideoCapture。";
  setMessage("");
}

async function request(path, options = {}) {
  const response = await fetch(path, { cache: "no-store", ...options });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || "请求失败");
  return data;
}

async function start() {
  $("startButton").disabled = true;
  setMessage("正在启动检测程序…", true);
  try {
    const data = await request("/api/start", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ source, value: $("sourceValue").value.trim(),
        calibrate: $("calibrateToggle").checked, loop: $("loopToggle").checked }),
    });
    processState = data.state;
    setMessage(data.calibrated ? "已启动，正在等待第一帧。" : "已启动。若勾选标定，请在弹出的 OpenCV 窗口完成标点。", true);
    addEvent("已启动 " + sourceNames[source], data.calibrated ? "加载既有标定" : "等待标定或相机帧");
    await pollStatus();
  } catch (error) {
    setMessage(error.message);
    addEvent("启动失败", error.message, "error");
    $("startButton").disabled = false;
  }
}

async function stop() {
  $("stopButton").disabled = true;
  try {
    await request("/api/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    setMessage("已停止分析。", true);
    addEvent("分析已停止", "进程与相机已关闭", "neutral");
    await pollStatus();
  } catch (error) {
    setMessage(error.message);
  }
}

async function upload(file) {
  setMessage(`正在导入 ${file.name}…`, true);
  $("browseButton").disabled = true;
  try {
    const response = await fetch("/api/upload", { method: "POST",
      headers: { "Content-Type": "application/octet-stream", "X-Filename": encodeURIComponent(file.name) }, body: file });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "上传失败");
    sourceValues.recording = data.path;
    $("sourceValue").value = data.path;
    setMessage(`已导入 ${file.name}，可以启动回放。`, true);
    addEvent("录像已导入", file.name);
  } catch (error) {
    setMessage(error.message);
    addEvent("录像导入失败", error.message, "error");
  } finally {
    $("browseButton").disabled = false;
    $("filePicker").value = "";
  }
}

function renderStatus(data) {
  processState = data.state;
  const running = data.state === "running";
  const active = running || data.state === "preparing";
  $("startButton").disabled = active || !data.binary_ready;
  $("stopButton").disabled = !active;
  document.querySelectorAll(".source-tab").forEach((button) => button.disabled = active);
  $("sourceValue").disabled = active;
  $("browseButton").disabled = active;
  $("calibrateToggle").disabled = active;
  $("loopToggle").disabled = active;
  if (firstStatus && !data.calibrated) $("calibrateToggle").checked = true;
  firstStatus = false;

  const labels = { idle: "未连接", running: "分析运行中", preparing: "模型准备中", finished: "回放结束", failed: "程序异常", stopped: "已停止" };
  $("globalStatus").className = "status-pill " + (active ? "running" : data.state === "failed" ? "failed" : "");
  $("globalStatus").innerHTML = `<i></i>${labels[data.state] || "未知状态"}`;
  if (!currentState || Date.now() - currentState.timestamp_ms > 2500) {
    $("metricStream").textContent = data.state === "preparing" ? "重建模型" : running ? "等待首帧" : data.state === "finished" ? "回放结束" : data.state === "failed" ? "运行异常" : "等待接入";
    $("metricSource").textContent = data.source ? sourceNames[data.source] : "尚未选择运行中的信号";
    $("metricCalibration").textContent = data.calibrated ? "已标定" : "待标定";
    $("metricCalibrationFoot").textContent = data.calibrated ? "相机位姿已保存在 config.yml" : "未取得相机位姿";
  }
  if (data.state === "failed" && lastStatus !== "failed") {
    const reason = data.log?.split("\n").filter(Boolean).slice(-1)[0] || `退出码 ${data.exit_code}`;
    setMessage("程序异常退出：" + reason);
    addEvent("程序异常退出", reason, "error");
  } else if (data.state === "finished" && lastStatus === "running") {
    addEvent("录像回放结束", "最后一帧保留在画面中", "neutral");
  } else if (data.state === "preparing" && lastStatus !== "preparing") {
    setMessage("现有 TensorRT 引擎不兼容，正在自动重建。", true);
    addEvent("正在重建推理引擎", "已备份旧引擎，请稍候", "warn");
  } else if (data.state === "running" && lastStatus === "preparing") {
    setMessage("推理引擎准备完成，正在载入录像。", true);
    addEvent("推理引擎已就绪", "继续启动相机分析");
  }
  lastStatus = data.state;
}

async function pollStatus() {
  try { renderStatus(await request("/api/status")); }
  catch (error) { setMessage("本地服务暂时不可用：" + error.message); }
}

function teamOf(id) { return id < 0 ? "unknown" : id < 6 ? "blue" : "red"; }
function robotName(id) {
  if (id < 0) return "未识别装甲";
  const names = ["英雄", "工程", "步兵 3", "步兵 4", "步兵 5", "哨兵"];
  const index = id % 6;
  return (id < 6 ? "蓝方 " : "红方 ") + (names[index] || `类别 ${id}`);
}
function shortName(target) { return target.robot_id < 0 ? `T${target.track_id}` : `${target.robot_id < 6 ? "B" : "R"}${target.robot_id % 6 + 1}`; }
function n(value, digits = 1) { return Number(value).toFixed(digits); }

function syncFrameGeometry() {
  const state = currentState;
  if (!state?.width || !state?.height) return;
  const stage = $("videoStage").getBoundingClientRect();
  const ratio = Math.min(stage.width / state.width, stage.height / state.height);
  const width = state.width * ratio, height = state.height * ratio;
  Object.assign($("boxes").style, {
    left: `${(stage.width - width) / 2}px`, top: `${(stage.height - height) / 2}px`,
    width: `${width}px`, height: `${height}px`, right: "auto", bottom: "auto",
  });
}

function renderTargets(targets, state) {
  const boxes = $("boxes"), markers = $("mapMarkers"), rows = $("targetRows");
  boxes.replaceChildren(); markers.replaceChildren(); rows.replaceChildren();
  const fieldWidth = state.field_width || 28, fieldHeight = state.field_height || 15;
  let mapped = 0;
  for (const target of targets) {
    const id = Number(target.track_id), box = target.bbox || [];
    const team = teamOf(target.robot_id);
    const selected = id === selectedId;
    if (box.length === 4 && state.width > 0 && state.height > 0 && box.every(Number.isFinite)) {
      const rect = document.createElement("div");
      rect.className = "detection-box" + (selected ? " selected" : "");
      Object.assign(rect.style, { left: `${Math.max(0, box[0] / state.width * 100)}%`,
        top: `${Math.max(0, box[1] / state.height * 100)}%`,
        width: `${Math.max(0, Math.min(100, box[2] / state.width * 100))}%`,
        height: `${Math.max(0, Math.min(100, box[3] / state.height * 100))}%` });
      const label = document.createElement("span");
      label.className = "detection-label";
      label.textContent = `#${id}  ${shortName(target)}  ${Math.round(target.confidence * 100)}%`;
      rect.append(label); boxes.append(rect);
    }
    const position = target.position;
    const inField = Array.isArray(position) && position.length === 2 &&
      position.every(Number.isFinite) && position[0] >= 0 && position[0] <= fieldWidth && position[1] >= 0 && position[1] <= fieldHeight;
    if (inField) {
      mapped++;
      const marker = document.createElement("button");
      marker.type = "button";
      marker.className = `map-marker ${team === "unknown" ? "" : team}${selected ? " selected" : ""}`;
      marker.style.left = `${position[0] / fieldWidth * 100}%`;
      marker.style.top = `${(1 - position[1] / fieldHeight) * 100}%`;
      marker.textContent = shortName(target);
      marker.title = `${robotName(target.robot_id)} #${id} · (${n(position[0])}, ${n(position[1])}) m`;
      marker.addEventListener("click", () => selectTarget(id));
      markers.append(marker);
    }
    const row = document.createElement("tr");
    if (selected) row.className = "selected";
    const idCell = document.createElement("td");
    const token = document.createElement("span"); token.className = "id-token"; token.textContent = String(id); idCell.append(token);
    const role = document.createElement("td"); role.textContent = robotName(target.robot_id);
    const confidence = document.createElement("td");
    confidence.textContent = `${Math.round(target.confidence * 100)}%`;
    const track = document.createElement("span"); track.className = "confidence-track";
    const bar = document.createElement("i"); bar.style.width = `${Math.max(0, Math.min(100, target.confidence * 100))}%`;
    track.append(bar); confidence.append(track);
    const pixel = document.createElement("td"); pixel.textContent = Array.isArray(target.pixel) ? `${Math.round(target.pixel[0])}, ${Math.round(target.pixel[1])} px` : "—";
    const coordinates = document.createElement("td");
    coordinates.className = position ? "estimate" : "unavailable";
    coordinates.textContent = position ? `${n(position[0])}, ${n(position[1])} m` : "待标定";
    const origin = document.createElement("td"); origin.textContent = position ? "单目 PnP 估计" : "仅像素跟踪";
    row.append(idCell, role, confidence, pixel, coordinates, origin);
    row.addEventListener("click", () => selectTarget(id)); rows.append(row);
  }
  if (!targets.length) {
    const row = document.createElement("tr"); row.className = "table-empty";
    const cell = document.createElement("td"); cell.colSpan = 6; cell.textContent = "当前没有有效目标";
    row.append(cell); rows.append(row);
  }
  $("mapCount").textContent = `${mapped} / ${targets.length} 定位`;
  $("targetCountChip").textContent = `${targets.length} 个目标`;
  $("metricTargets").innerHTML = `${String(targets.length).padStart(2, "0")}<span class="metric-unit">个</span>`;
  syncFrameGeometry();
}

function selectTarget(id) {
  selectedId = selectedId === id ? null : id;
  if (currentState) renderTargets(activeTargets(currentState), currentState);
}

function activeTargets(data) {
  return data && data.result_age_ms <= 1200 && Date.now() - data.timestamp_ms <= 2500 ? data.targets || [] : [];
}

function renderFrame(data) {
  currentState = data;
  const fresh = Date.now() - data.timestamp_ms <= 2500;
  const targets = activeTargets(data);
  if (selectedId !== null && !targets.some((target) => target.track_id === selectedId)) selectedId = null;
  if (data.sequence !== currentSequence) {
    currentSequence = data.sequence;
    $("videoFrame").src = `/api/frame?sequence=${data.sequence}`;
    $("videoStage").classList.add("has-frame");
    $("videoResolution").textContent = `${data.width} × ${data.height}`;
    $("frameSequence").textContent = `FRAME ${String(data.sequence).padStart(5, "0")}`;
    $("frameTime").textContent = new Date(data.timestamp_ms).toLocaleTimeString("zh-CN", { hour12: false });
    const ids = new Set(targets.map((target) => target.track_id));
    const incoming = targets.filter((target) => !lastTargetIds.has(target.track_id));
    for (const target of incoming.slice(0, 2)) addEvent(`发现轨迹 #${target.track_id}`, robotName(target.robot_id));
    lastTargetIds = ids;
  }
  $("videoLive").className = "live-chip " + (fresh ? "live" : "idle");
  $("videoLive").innerHTML = `<i></i>${fresh ? "实时画面" : "最后一帧"}`;
  $("metricStream").textContent = fresh ? "画面接入" : "信号中断";
  $("metricSource").textContent = data.source === "recording" ? "相机内录视频" : "实时相机画面";
  $("metricLatency").innerHTML = `${Number.isFinite(data.latency_ms) ? data.latency_ms : "--"}<span class="metric-unit">ms</span>`;
  $("metricCalibration").textContent = data.calibrated ? "已标定" : "待标定";
  $("metricCalibrationFoot").textContent = data.calibrated ? "基于 PnP 的场地坐标估计" : "当前仅提供像素位置";
  $("mapBadge").className = "map-badge " + (data.calibrated ? "ready" : "");
  $("mapBadge").textContent = data.calibrated ? "单目估计" : "等待标定";
  $("mapNoCalibration").classList.toggle("hidden", Boolean(data.calibrated));
  renderTargets(targets, data);
}

async function pollState() {
  try {
    const data = await request("/api/state");
    if (data.sequence) renderFrame(data);
  } catch (error) {
    if (processState === "running") setMessage("读取实时数据失败：" + error.message);
  }
}

document.querySelectorAll(".source-tab").forEach((button) => button.addEventListener("click", () => setSource(button.dataset.source)));
$("startButton").addEventListener("click", start);
$("stopButton").addEventListener("click", stop);
$("browseButton").addEventListener("click", () => $("filePicker").click());
$("filePicker").addEventListener("change", (event) => { if (event.target.files[0]) upload(event.target.files[0]); });
$("videoFrame").addEventListener("error", () => { if (processState === "running") setMessage("视频帧暂不可用，正在重试。", true); });
new ResizeObserver(syncFrameGeometry).observe($("videoStage"));
setInterval(() => { $("clock").textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false }); }, 1000);
$("clock").textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false });
pollStatus(); pollState();
setInterval(pollStatus, 1700);
setInterval(pollState, 350);
