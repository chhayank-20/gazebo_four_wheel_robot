/**
 * Autonomous Office Robot — Web Chat & Telemetry Controller
 * Connects to FastAPI backend (/chat, /status, /health, /locations, /commands)
 */

(function () {
  "use strict";

  // --- State & Session ---
  const API_BASE = "";
  let conversationId = sessionStorage.getItem("robot_chat_conv_id");
  if (!conversationId) {
    conversationId = "conv_" + Date.now().toString(36) + "_" + Math.random().toString(36).substring(2, 7);
    sessionStorage.setItem("robot_chat_conv_id", conversationId);
  }

  let isSending = false;
  let activeCommandId = null;
  let pollIntervalId = null;

  // --- DOM Elements ---
  const messagesContainer = document.getElementById("messages-container");
  const chatForm = document.getElementById("chat-form");
  const messageInput = document.getElementById("message-input");
  const sendBtn = document.getElementById("send-btn");
  const typingIndicator = document.getElementById("typing-indicator");
  const errorToast = document.getElementById("error-toast");
  const errorToastMsg = document.getElementById("error-toast-msg");
  const closeToastBtn = document.getElementById("close-toast-btn");
  const clearChatBtn = document.getElementById("clear-chat-btn");
  const toggleTelemetryBtn = document.getElementById("toggle-telemetry-btn");
  const telemetrySidebar = document.getElementById("telemetry-sidebar");

  // Top Nav Elements
  const connectionPill = document.getElementById("connection-pill");
  const connectionText = document.getElementById("connection-text");

  // Health Elements
  const healthBadge = document.getElementById("health-badge");
  const hRosVal = document.getElementById("h-ros-val");
  const hNavVal = document.getElementById("h-nav-val");
  const hOdomVal = document.getElementById("h-odom-val");
  const hScanVal = document.getElementById("h-scan-val");
  const hTfVal = document.getElementById("h-tf-val");

  // Telemetry Elements
  const poseX = document.getElementById("pose-x");
  const poseY = document.getElementById("pose-y");
  const poseYaw = document.getElementById("pose-yaw");
  const velLinear = document.getElementById("vel-linear");
  const velAngular = document.getElementById("vel-angular");
  const lidarVal = document.getElementById("lidar-val");
  const clearanceBar = document.getElementById("clearance-bar");
  const clearanceStatus = document.getElementById("clearance-status");
  const telemetryFreshness = document.getElementById("telemetry-freshness");

  // Mission Card Elements
  const activeMissionCard = document.getElementById("active-mission-card");
  const missionDestName = document.getElementById("mission-dest-name");
  const missionCmdId = document.getElementById("mission-cmd-id");
  const missionDistance = document.getElementById("mission-distance");
  const missionStatusBadge = document.getElementById("mission-status-badge");
  const cancelMissionBtn = document.getElementById("cancel-mission-btn");

  // Locations List
  const locationsList = document.getElementById("locations-list");
  const locationsCount = document.getElementById("locations-count");

  // Simulation & Camera Elements
  const simSection = document.getElementById("sim-section");
  const simFrame = document.getElementById("sim-frame");
  const simOverlay = document.getElementById("sim-overlay");
  const camModeBadge = document.getElementById("cam-mode-badge");
  const camButtons = document.querySelectorAll(".cam-btn[data-mode]");
  const fullscreenSimBtn = document.getElementById("btn-fullscreen-sim");
  const mobileTabBtns = document.querySelectorAll(".tab-btn[data-target]");

  // --- Initial Setup ---
  document.getElementById("initial-timestamp").textContent = formatTime(new Date());

  // --- Utilities ---
  function formatTime(date) {
    return date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  }

  function escapeHtml(text) {
    if (!text) return "";
    return text
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;")
      .replace(/'/g, "&#039;");
  }

  function formatMarkdown(text) {
    if (!text) return "";
    let safe = escapeHtml(text);
    // Bold
    safe = safe.replace(/\*\*(.*?)\*\*/g, "<strong>$1</strong>");
    // Inline code
    safe = safe.replace(/`([^`]+)`/g, "<code>$1</code>");
    // Bullet lists
    safe = safe.replace(/^[*-]\s+(.+)$/gm, "<li>$1</li>");
    safe = safe.replace(/(<li>[\s\S]*?<\/li>)/gm, "<ul>$1</ul>");
    // Clean up empty lines
    safe = safe.replace(/\n\n/g, "</p><p>");
    return `<p>${safe}</p>`;
  }

  function showToast(msg, durationMs = 5000) {
    errorToastMsg.textContent = msg;
    errorToast.classList.remove("hidden");
    if (durationMs > 0) {
      setTimeout(() => {
        errorToast.classList.add("hidden");
      }, durationMs);
    }
  }

  closeToastBtn.addEventListener("click", () => {
    errorToast.classList.add("hidden");
  });

  // --- Auto-resize Textarea ---
  messageInput.addEventListener("input", function () {
    this.style.height = "auto";
    this.style.height = Math.min(this.scrollHeight, 120) + "px";
  });

  messageInput.addEventListener("keydown", function (e) {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      if (!isSending) {
        chatForm.dispatchEvent(new Event("submit"));
      }
    }
  });

  // --- Chat Submission ---
  chatForm.addEventListener("submit", async function (e) {
    e.preventDefault();
    const text = messageInput.value.trim();
    if (!text || isSending) return;

    // Send user message
    await submitUserMessage(text);
  });

  async function submitUserMessage(text) {
    if (isSending) return;

    // Prevent duplicate submission & lock input
    isSending = true;
    messageInput.value = "";
    messageInput.style.height = "auto";
    messageInput.disabled = true;
    sendBtn.disabled = true;

    // Append User Message to UI
    appendUserMessage(text);
    showTyping(true);

    try {
      const response = await fetch(`${API_BASE}/chat`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          message: text,
          conversation_id: conversationId,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        const errorDetail = data.detail || `Server error (${response.status})`;
        appendAssistantMessage(
          `⚠️ **Request Error (${response.status})**: ${errorDetail}`,
          null,
          null
        );
        showToast(errorDetail);
        return;
      }

      // Handle successful chat response
      const assistantText = data.response || "I have processed your request.";
      const toolCalls = data.tool_calls || [];
      const cmdId = data.command_id;

      appendAssistantMessage(assistantText, toolCalls, cmdId);

      // If a command was accepted / dispatched, update active mission state
      if (cmdId) {
        activeCommandId = cmdId;
        fetchRobotStatus(); // Trigger immediate status refresh
      }
    } catch (err) {
      console.error("Chat network error:", err);
      const networkError = "Network error: Unable to reach the robot API server. Please check your connection.";
      appendAssistantMessage(`❌ **Connection Failed**: ${networkError}`, null, null);
      showToast(networkError);
    } finally {
      showTyping(false);
      isSending = false;
      messageInput.disabled = false;
      sendBtn.disabled = false;
      messageInput.focus();
    }
  }

  function appendUserMessage(text) {
    const timeStr = formatTime(new Date());
    const wrapper = document.createElement("div");
    wrapper.className = "message-wrapper user";
    wrapper.innerHTML = `
      <div class="avatar">👤</div>
      <div class="message-bubble">
        <div class="message-meta">
          <span class="sender-name">Operator</span>
          <span class="timestamp">${timeStr}</span>
        </div>
        <div class="message-content">
          <p>${escapeHtml(text)}</p>
        </div>
      </div>
    `;
    messagesContainer.appendChild(wrapper);
    scrollToBottom();
  }

  function appendAssistantMessage(text, toolCalls, commandId) {
    const timeStr = formatTime(new Date());
    const wrapper = document.createElement("div");
    wrapper.className = "message-wrapper assistant";

    let html = `
      <div class="avatar">🤖</div>
      <div class="message-bubble">
        <div class="message-meta">
          <span class="sender-name">Robot Assistant</span>
          <span class="timestamp">${timeStr}</span>
        </div>
        <div class="message-content">
          ${formatMarkdown(text)}
        </div>
    `;

    // Ambiguity Clarification quick buttons
    const lower = text.toLowerCase();
    if (lower.includes("experience centre has three distinct") || lower.includes("which one would you like to visit")) {
      html += `
        <div class="clarification-options">
          <button class="clarification-btn" data-dest="living_demo">Living Demo Room</button>
          <button class="clarification-btn" data-dest="hotel_exp">Hotel Room</button>
          <button class="clarification-btn" data-dest="hospital_exp">Hospital Room</button>
        </div>
      `;
    }

    // Tool executions inspection box
    if (toolCalls && toolCalls.length > 0) {
      html += `<div class="tool-calls-container">`;
      toolCalls.forEach((tc) => {
        const tName = tc.tool_name || "tool";
        const argsStr = JSON.stringify(tc.arguments || {}, null, 2);
        const resStr = JSON.stringify(tc.result || {}, null, 2);
        html += `
          <div class="tool-call-box">
            <div class="tool-call-header" onclick="this.nextElementSibling.classList.toggle('hidden')">
              <span>🔧 Executed Tool: <strong>${escapeHtml(tName)}</strong></span>
              <span style="font-size: 0.7em;">▼ Toggle details</span>
            </div>
            <div class="tool-details hidden">
              <strong>Arguments:</strong>\n${escapeHtml(argsStr)}\n
              <strong>Result:</strong>\n${escapeHtml(resStr)}
            </div>
          </div>
        `;
      });
      html += `</div>`;
    }

    html += `</div>`;
    wrapper.innerHTML = html;
    messagesContainer.appendChild(wrapper);

    // Bind clarification buttons inside this message
    wrapper.querySelectorAll(".clarification-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        const destId = btn.getAttribute("data-dest");
        const destLabel = btn.textContent;
        submitUserMessage(`Please navigate to the ${destLabel}`);
      });
    });

    scrollToBottom();
  }

  function showTyping(show) {
    if (show) {
      typingIndicator.classList.remove("hidden");
      scrollToBottom();
    } else {
      typingIndicator.classList.add("hidden");
    }
  }

  function scrollToBottom() {
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  // Quick Chips inside welcome message
  document.addEventListener("click", function (e) {
    if (e.target && e.target.classList.contains("chip")) {
      const prompt = e.target.getAttribute("data-prompt");
      if (prompt) {
        submitUserMessage(prompt);
      }
    }
  });

  // Clear Chat History
  clearChatBtn.addEventListener("click", () => {
    // Keep initial welcome message
    const initialWelcome = messagesContainer.firstElementChild;
    messagesContainer.innerHTML = "";
    if (initialWelcome) {
      messagesContainer.appendChild(initialWelcome);
    }
  });

  // Mobile Telemetry Toggle
  toggleTelemetryBtn.addEventListener("click", () => {
    telemetrySidebar.classList.toggle("active");
    toggleTelemetryBtn.textContent = telemetrySidebar.classList.contains("active") ? "✕ Close" : "📊 Telemetry";
  });

  // --- Telemetry & Health Polling ---
  async function fetchRobotHealth() {
    try {
      const res = await fetch(`${API_BASE}/health`);
      if (!res.ok) throw new Error(`Health status: ${res.status}`);
      const data = await res.json();

      // Top pill
      connectionPill.className = "pill";
      if (data.overall_status === "healthy") {
        connectionPill.classList.add("pill-healthy");
        connectionText.textContent = "Live & Healthy";
        healthBadge.className = "badge badge-healthy";
        healthBadge.textContent = "HEALTHY";
      } else if (data.overall_status === "degraded") {
        connectionPill.classList.add("pill-degraded");
        connectionText.textContent = "Degraded Feeds";
        healthBadge.className = "badge badge-degraded";
        healthBadge.textContent = "DEGRADED";
      } else {
        connectionPill.classList.add("pill-offline");
        connectionText.textContent = "Unhealthy";
        healthBadge.className = "badge badge-unhealthy";
        healthBadge.textContent = "UNHEALTHY";
      }

      // Detailed subsystems
      setHealthItem(hRosVal, data.ros2_connected);
      setHealthItem(hNavVal, data.nav2_action_server_ready);
      setHealthItem(hOdomVal, data.odom_active);
      setHealthItem(hScanVal, data.scan_active);
      setHealthItem(hTfVal, data.tf_active);
    } catch (err) {
      connectionPill.className = "pill pill-offline";
      connectionText.textContent = "API Disconnected";
      healthBadge.className = "badge badge-unhealthy";
      healthBadge.textContent = "OFFLINE";
      setHealthItem(hRosVal, false);
      setHealthItem(hNavVal, false);
      setHealthItem(hOdomVal, false);
      setHealthItem(hScanVal, false);
      setHealthItem(hTfVal, false);
    }
  }

  function setHealthItem(elem, isOk) {
    if (isOk === true) {
      elem.textContent = "OK";
      elem.className = "h-status ok";
    } else {
      elem.textContent = "OFFLINE";
      elem.className = "h-status fail";
    }
  }

  async function fetchRobotStatus() {
    try {
      const res = await fetch(`${API_BASE}/status`);
      if (!res.ok) return;
      const data = await res.json();

      telemetryFreshness.textContent = "Updated: " + formatTime(new Date());

      // 1. Pose
      if (data.pose) {
        poseX.textContent = Number(data.pose.x).toFixed(2);
        poseY.textContent = Number(data.pose.y).toFixed(2);
        const yawDeg = ((data.pose.yaw * 180) / Math.PI).toFixed(0);
        poseYaw.textContent = `${yawDeg}°`;
        poseX.style.color = "";
        poseY.style.color = "";
        poseYaw.style.color = "";
      } else {
        poseX.textContent = "None";
        poseY.textContent = "None";
        poseYaw.textContent = "None";
        poseX.style.color = "var(--warning)";
        poseY.style.color = "var(--warning)";
        poseYaw.style.color = "var(--warning)";
      }

      // 2. Velocity
      if (data.velocity) {
        velLinear.textContent = Number(data.velocity.linear_x).toFixed(2);
        velAngular.textContent = Number(data.velocity.angular_z).toFixed(2);
      } else {
        velLinear.textContent = "None";
        velAngular.textContent = "None";
      }

      // 3. LiDAR Clearance
      if (data.lidar && data.lidar.min_lidar_m !== null && data.lidar.min_lidar_m !== undefined) {
        const m = Number(data.lidar.min_lidar_m);
        lidarVal.textContent = m.toFixed(2);
        const pct = Math.min(100, Math.max(5, (m / 3.0) * 100));
        clearanceBar.style.width = pct + "%";

        if (m < 0.5) {
          clearanceBar.style.backgroundColor = "var(--danger)";
          clearanceStatus.textContent = "Obstacle Close (<0.5m)";
          clearanceStatus.style.color = "var(--danger)";
        } else if (m < 1.0) {
          clearanceBar.style.backgroundColor = "var(--warning)";
          clearanceStatus.textContent = "Caution (0.5-1.0m)";
          clearanceStatus.style.color = "var(--warning)";
        } else {
          clearanceBar.style.backgroundColor = "var(--success)";
          clearanceStatus.textContent = "Clear (>1.0m)";
          clearanceStatus.style.color = "var(--success)";
        }
      } else {
        lidarVal.textContent = "None";
        clearanceBar.style.width = "0%";
        clearanceStatus.textContent = "Stale / No Scan";
        clearanceStatus.style.color = "var(--warning)";
      }

      // 4. Active Navigation Mission Card
      updateActiveMissionCard(data.active_command);
    } catch (err) {
      console.warn("Failed to fetch live telemetry:", err);
    }
  }

  function updateActiveMissionCard(activeCmd) {
    if (!activeCmd) {
      // If we previously had an active command that finished, mark complete
      if (activeCommandId) {
        missionStatusBadge.className = "badge badge-healthy";
        missionStatusBadge.textContent = "FINISHED";
        setTimeout(() => {
          activeMissionCard.classList.add("hidden");
          activeCommandId = null;
        }, 5000);
      } else {
        activeMissionCard.classList.add("hidden");
      }
      return;
    }

    activeCommandId = activeCmd.command_id;
    activeMissionCard.classList.remove("hidden");

    missionDestName.textContent = activeCmd.destination_name || activeCmd.location_id;
    missionCmdId.textContent = activeCmd.command_id;

    const st = (activeCmd.status || "RUNNING").toUpperCase();
    missionStatusBadge.textContent = st;
    if (st === "RUNNING" || st === "ACCEPTED") {
      missionStatusBadge.className = "badge badge-primary";
      cancelMissionBtn.disabled = false;
      cancelMissionBtn.textContent = "🛑 Cancel / Stop Navigation";
    } else if (st === "SUCCEEDED") {
      missionStatusBadge.className = "badge badge-healthy";
      cancelMissionBtn.disabled = true;
    } else {
      missionStatusBadge.className = "badge badge-unhealthy";
      cancelMissionBtn.disabled = true;
    }

    if (activeCmd.distance_remaining_m !== null && activeCmd.distance_remaining_m !== undefined) {
      missionDistance.textContent = Number(activeCmd.distance_remaining_m).toFixed(2) + " m";
    } else {
      missionDistance.textContent = "Calculating...";
    }
  }

  // --- Cancellation Button Handler ---
  cancelMissionBtn.addEventListener("click", async function () {
    if (!activeCommandId) return;

    cancelMissionBtn.disabled = true;
    cancelMissionBtn.textContent = "⏳ Stopping Robot...";

    try {
      const res = await fetch(`${API_BASE}/commands/${activeCommandId}/cancel`, {
        method: "POST",
      });
      const data = await res.json();

      if (res.ok && data.verified_stopped) {
        showToast("Navigation stopped. Verified robot at full rest.", 4000);
        appendAssistantMessage(
          `🛑 **Navigation Canceled**: Command \`${activeCommandId}\` stopped successfully. Stationary rest verified.`,
          null,
          null
        );
      } else {
        showToast("Cancellation notice: " + (data.detail || data.message || "Failed"), 5000);
      }
    } catch (err) {
      showToast("Cancellation request error: " + err.message);
    } finally {
      fetchRobotStatus();
    }
  });

  // --- Approved Destinations List ---
  async function fetchApprovedLocations() {
    try {
      const res = await fetch(`${API_BASE}/locations`);
      if (!res.ok) throw new Error(`Locations status: ${res.status}`);
      const data = await res.json();
      const locs = data.locations || [];

      locationsCount.textContent = `${locs.length} Approved`;
      locationsList.innerHTML = "";

      locs.forEach((loc) => {
        const item = document.createElement("div");
        item.className = "location-item";
        item.innerHTML = `
          <div>
            <div class="loc-name">${escapeHtml(loc.name)}</div>
            <div class="loc-id">${escapeHtml(loc.id)} · (${Number(loc.x).toFixed(1)}, ${Number(loc.y).toFixed(1)})</div>
          </div>
          <span class="loc-action">Go ➔</span>
        `;
        item.addEventListener("click", () => {
          submitUserMessage(`Please navigate to the ${loc.name}`);
        });
        locationsList.appendChild(item);
      });
    } catch (err) {
      locationsList.innerHTML = `<div class="location-item text-muted">Unable to load locations</div>`;
    }
  }

  // --- Gazebo 3D Simulation & noVNC Integration ---
  function initSimulationViewport() {
    if (!simFrame) return;

    // Build the dynamic noVNC URL pointing to port 8080 on the current host
    const host = window.location.hostname || "localhost";
    const vncUrl = `${window.location.protocol}//${host}:8080/vnc.html?autoconnect=true&resize=scale&reconnect=true&reconnect_delay=1500`;

    // Only set src if not already set or blank
    if (!simFrame.src || simFrame.src === "about:blank" || simFrame.src.endsWith("about:blank")) {
      simFrame.src = vncUrl;
    }

    // Hide loading overlay once iframe loads
    simFrame.addEventListener("load", () => {
      if (simOverlay) {
        simOverlay.classList.add("hidden");
      }
    });

    // Fallback: hide overlay after 2.5 seconds in case iframe load event is suppressed
    setTimeout(() => {
      if (simOverlay) {
        simOverlay.classList.add("hidden");
      }
    }, 2500);

    // Update external links dynamically
    document.querySelectorAll(".vnc-popout-btn").forEach((btn) => {
      btn.href = `http://${host}:8080/vnc.html?autoconnect=true&resize=remote`;
    });
  }

  // --- Gazebo Camera Controls ---
  const CAM_MODE_LABELS = {
    follow: "FOLLOW (ROBOT)",
    top_down: "TOP-DOWN",
    free: "FREE (ORBIT)",
    reset: "RESET",
  };

  function updateCameraUI(activeMode) {
    if (camModeBadge) {
      const label = CAM_MODE_LABELS[activeMode] || activeMode.toUpperCase();
      camModeBadge.textContent = `CAMERA: ${label}`;
    }

    camButtons.forEach((btn) => {
      if (btn.dataset.mode === activeMode) {
        btn.classList.add("active");
      } else {
        btn.classList.remove("active");
      }
    });
  }

  async function setCameraMode(mode) {
    updateCameraUI(mode);

    try {
      const res = await fetch(`${API_BASE}/camera/mode`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: mode }),
      });

      const data = await res.json();
      if (!res.ok) {
        showToast(data.detail || `Camera control error (${res.status})`, 4000);
        return;
      }

      if (data.current_mode) {
        updateCameraUI(data.current_mode);
      }
    } catch (err) {
      console.warn("Failed to set camera mode:", err);
      showToast("Camera service unreachable: " + err.message, 4000);
    }
  }

  async function fetchCameraMode() {
    try {
      const res = await fetch(`${API_BASE}/camera/mode`);
      if (res.ok) {
        const data = await res.json();
        if (data.current_mode) {
          updateCameraUI(data.current_mode);
        }
      }
    } catch (err) {
      // Quietly ignore camera mode polling errors
    }
  }

  function setupCameraControls() {
    camButtons.forEach((btn) => {
      btn.addEventListener("click", () => {
        const mode = btn.dataset.mode;
        if (mode) {
          setCameraMode(mode);
        }
      });
    });
  }

  // --- Fullscreen / Viewport Expand ---
  function setupFullscreenToggle() {
    if (!fullscreenSimBtn || !simSection) return;

    fullscreenSimBtn.addEventListener("click", () => {
      const isFull = simSection.classList.toggle("fullscreen-active");
      if (isFull) {
        fullscreenSimBtn.textContent = "↙ Collapse";
        fullscreenSimBtn.title = "Collapse Simulation Viewport";
      } else {
        fullscreenSimBtn.textContent = "⛶ Expand";
        fullscreenSimBtn.title = "Toggle Fullscreen Viewport";
      }
    });

    // Allow Escape key to exit fullscreen
    document.addEventListener("keydown", (e) => {
      if (e.key === "Escape" && simSection.classList.contains("fullscreen-active")) {
        simSection.classList.remove("fullscreen-active");
        fullscreenSimBtn.textContent = "⛶ Expand";
        fullscreenSimBtn.title = "Toggle Fullscreen Viewport";
      }
    });
  }

  // --- Mobile Tab Navigation ---
  function setupMobileTabs() {
    if (!mobileTabBtns || mobileTabBtns.length === 0) return;

    mobileTabBtns.forEach((tabBtn) => {
      tabBtn.addEventListener("click", () => {
        const targetId = tabBtn.dataset.target;
        if (!targetId) return;

        // Switch active tab button
        mobileTabBtns.forEach((b) => b.classList.remove("active"));
        tabBtn.classList.add("active");

        // Switch active content pane
        const panes = document.querySelectorAll(".main-layout > aside, .main-layout > section");
        panes.forEach((pane) => pane.classList.remove("tab-active"));

        const targetPane = document.getElementById(targetId);
        if (targetPane) {
          targetPane.classList.add("tab-active");
        }
      });
    });
  }

  // --- Initialize Polling Loop ---
  function startPolling() {
    initSimulationViewport();
    setupCameraControls();
    setupFullscreenToggle();
    setupMobileTabs();

    fetchRobotHealth();
    fetchRobotStatus();
    fetchApprovedLocations();
    fetchCameraMode();

    pollIntervalId = setInterval(() => {
      fetchRobotHealth();
      fetchRobotStatus();
    }, 1500);
  }

  startPolling();
})();

