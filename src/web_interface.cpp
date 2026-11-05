#include "web_interface.h"
#include <stdlib.h>
#include <string.h>

namespace {

const IPAddress AP_LOCAL_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>RC Rover HUD &amp; Control</title>
  <style>
    :root {
      color-scheme: dark;
      --bg-color: #0f172a;
      --card-bg: #1e293b;
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --brand-color: #38bdf8;
      --accent-green: #22c55e;
      --accent-amber: #f59e0b;
      --accent-red: #ef4444;
    }

    * {
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
    }

    body {
      margin: 0;
      padding: 12px;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background-color: var(--bg-color);
      color: var(--text-main);
      accent-color: var(--brand-color);
      user-select: none;
      -webkit-user-select: none;
    }

    .container {
      max-width: 440px;
      margin: 0 auto;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }

    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 10px 14px;
      background: var(--card-bg);
      border-radius: 12px;
    }

    .header h1 {
      margin: 0;
      font-size: 1.05rem;
      letter-spacing: 0.02em;
    }

    .conn-badge {
      font-size: 0.75rem;
      font-weight: 600;
      padding: 4px 10px;
      border-radius: 999px;
      background: #334155;
      color: var(--text-muted);
    }

    .conn-badge.online {
      background: rgba(34, 197, 94, 0.2);
      color: var(--accent-green);
    }

    .card {
      background: var(--card-bg);
      border-radius: 12px;
      padding: 14px;
    }

    /* Live HUD / Camera Viewport */
    .hud-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
    }

    .cam-btn {
      border: 1px solid #334155;
      background: #0f172a;
      color: var(--brand-color);
      border-radius: 6px;
      padding: 4px 8px;
      font-size: 0.72rem;
      font-weight: 600;
      cursor: pointer;
    }

    .cam-btn:focus-visible {
      outline: 2px solid var(--brand-color);
      outline-offset: 2px;
    }

    .hud-viewport {
      position: relative;
      width: 100%;
      height: 190px;
      background: #090d16;
      border-radius: 10px;
      overflow: hidden;
      border: 1px solid #334155;
    }

    #cam-stream {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      object-fit: cover;
      display: none;
    }

    #hud-canvas {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      cursor: crosshair;
    }

    .telemetry-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      align-items: center;
      margin-top: 10px;
    }

    .state-row {
      margin-top: 10px;
      padding-top: 10px;
      border-top: 1px solid #334155;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .metric-label {
      font-size: 0.72rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      color: var(--text-muted);
      margin-bottom: 2px;
    }

    .distance-readout {
      font-size: 1.6rem;
      font-weight: 700;
      font-variant-numeric: tabular-nums;
      color: var(--brand-color);
    }

    .state-readout {
      font-size: 0.85rem;
      font-weight: 600;
      padding: 5px 10px;
      border-radius: 8px;
      background: #0f172a;
      display: inline-block;
    }

    /* 3-Mode Selector Toggle Buttons */
    .mode-toggle {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 6px;
      background: #0f172a;
      padding: 4px;
      border-radius: 10px;
    }

    .mode-btn {
      border: 0;
      border-radius: 8px;
      padding: 10px 6px;
      font-size: 0.8rem;
      font-weight: 600;
      background: transparent;
      color: var(--text-muted);
      cursor: pointer;
    }

    .mode-btn.active {
      background: var(--brand-color);
      color: #0f172a;
    }

    .mode-btn:focus-visible {
      outline: 2px solid var(--brand-color);
      outline-offset: 2px;
    }

    /* Throttle Trim Range Slider */
    .slider-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 6px;
    }

    .slider-header label {
      font-size: 0.85rem;
      font-weight: 600;
    }

    .slider-value {
      font-size: 0.85rem;
      font-weight: 700;
      color: var(--brand-color);
      font-variant-numeric: tabular-nums;
    }

    input[type="range"] {
      --slider-track-size: 0.5rem;
      --slider-thumb-size: 1.5rem;
      --slider-track-radius: 999px;
      --slider-thumb-radius: 50%;
      --slider-track-color: #334155;
      --slider-fill-color: #38bdf8;
      --slider-thumb-color: var(--slider-fill-color);
      --slider-direction: to right;

      &:dir(rtl) {
        --slider-direction: to left;
      }

      --attr-min: attr(min type(<number>), 80);
      --attr-max: attr(max type(<number>), 255);
      --control-value: attr(value type(<number>), 170);
      --slider-progress: calc(
        progress(var(--control-value), var(--attr-min), var(--attr-max)) * 100%
      );

      -webkit-appearance: none;
      appearance: none;
      inline-size: 100%;
      width: 100%;
      block-size: 2.5rem;
      height: 2.5rem;
      margin: 0;
      padding: 0;
      border: 0;
      border-radius: 0;
      background: transparent;
      cursor: pointer;
    }

    @supports not (opacity: progress(1, 0, 100)) {
      input[type="range"] {
        --slider-progress: calc(
          (var(--control-value) - var(--attr-min)) /
          (var(--attr-max) - var(--attr-min)) * 100%
        );
      }
    }

    /* WebKit/Blink */
    input[type="range"]::-webkit-slider-runnable-track {
      block-size: var(--slider-track-size);
      height: var(--slider-track-size);
      border: 0;
      border-radius: var(--slider-track-radius);
      background: linear-gradient(
        var(--slider-direction),
        var(--slider-fill-color) var(--slider-progress),
        var(--slider-track-color) var(--slider-progress)
      );
    }

    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      inline-size: var(--slider-thumb-size);
      width: var(--slider-thumb-size);
      block-size: var(--slider-thumb-size);
      height: var(--slider-thumb-size);
      margin-block-start: calc(
        (var(--slider-track-size) - var(--slider-thumb-size)) / 2
      );
      margin-top: calc(
        (var(--slider-track-size) - var(--slider-thumb-size)) / 2
      );
      border: 0;
      border-radius: var(--slider-thumb-radius);
      background: var(--slider-thumb-color);
    }

    /* Firefox */
    input[type="range"]::-moz-range-track {
      block-size: var(--slider-track-size);
      height: var(--slider-track-size);
      border: 0;
      border-radius: var(--slider-track-radius);
      background: var(--slider-track-color);
    }

    input[type="range"]::-moz-range-progress {
      block-size: var(--slider-track-size);
      height: var(--slider-track-size);
      border-radius: var(--slider-track-radius);
      background: var(--slider-fill-color);
    }

    input[type="range"]::-moz-range-thumb {
      inline-size: var(--slider-thumb-size);
      width: var(--slider-thumb-size);
      block-size: var(--slider-thumb-size);
      height: var(--slider-thumb-size);
      border: 0;
      border-radius: var(--slider-thumb-radius);
      background: var(--slider-thumb-color);
    }

    input[type="range"]:focus-visible {
      outline: 2px solid var(--slider-fill-color);
      outline-offset: 4px;
    }

    /* Manual D-Pad Grid */
    .dpad-wrapper {
      transition: opacity 0.2s ease;
    }

    .dpad-wrapper.disabled {
      opacity: 0.4;
      pointer-events: none;
    }

    .dpad-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-top: 8px;
    }

    .pad-btn {
      border: 0;
      border-radius: 12px;
      height: 64px;
      font-size: 0.85rem;
      font-weight: 700;
      color: var(--text-main);
      background: #334155;
      cursor: pointer;
      touch-action: none;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 2px;
    }

    .pad-btn:active,
    .pad-btn.pressed {
      background: var(--brand-color);
      color: #0f172a;
    }

    .pad-btn.stop-btn {
      background: var(--accent-red);
      color: #fff;
    }

    .pad-btn:focus-visible {
      outline: 2px solid var(--brand-color);
      outline-offset: 2px;
    }

    .sub-hint {
      font-size: 0.7rem;
      color: var(--text-muted);
      text-align: center;
      margin-top: 6px;
    }
  </style>
</head>
<body>
  <main class="container">
    <header class="header">
      <h1>RC-ROVER S3</h1>
      <span id="conn-status" class="conn-badge">CONNECTING...</span>
    </header>

    <section class="card" aria-label="Vision &amp; Obstacle HUD">
      <div class="hud-header">
        <span class="metric-label" style="margin:0;">Live Obstacle &amp; Target HUD</span>
        <button type="button" id="btn-cam-toggle" class="cam-btn">Connect ESP32-CAM</button>
      </div>
      <div class="hud-viewport">
        <img id="cam-stream" crossorigin="anonymous" alt="ESP32-CAM Live Stream">
        <canvas id="hud-canvas" width="400" height="190"></canvas>
      </div>
      <div class="sub-hint" id="hud-hint">Tap anywhere in the HUD to lock/steer a Follow-Me target box, or view live sonar obstacle zones.</div>

      <div class="telemetry-grid">
        <div>
          <div class="metric-label">Front Sonar</div>
          <div id="front-dist-val" class="distance-readout">--.- cm</div>
        </div>
        <div>
          <div class="metric-label">Rear Sonar</div>
          <div id="rear-dist-val" class="distance-readout">--.- cm</div>
        </div>
      </div>
      <div class="state-row">
        <span class="metric-label" style="margin:0;">Navigation State</span>
        <span id="state-val" class="state-readout">INIT</span>
      </div>
    </section>

    <section class="card" aria-label="Drive Mode">
      <div class="metric-label" style="margin-bottom:8px;">Operation Mode</div>
      <div class="mode-toggle" role="group" aria-label="Operation Mode Toggle">
        <button type="button" id="btn-auto" class="mode-btn">Auto Steer</button>
        <button type="button" id="btn-follow" class="mode-btn">Follow Me</button>
        <button type="button" id="btn-manual" class="mode-btn">Manual D-Pad</button>
      </div>
    </section>

    <section class="card" aria-label="Throttle Trim">
      <div class="slider-header">
        <label for="throttle-slider">Throttle Trim (PWM)</label>
        <span id="throttle-readout" class="slider-value">170 / 255</span>
      </div>
      <input
        type="range"
        id="throttle-slider"
        min="80"
        max="255"
        step="5"
        value="170"
        aria-valuetext="PWM 170 of 255"
      >
    </section>

    <section id="dpad-section" class="card dpad-wrapper disabled" aria-label="Manual D-Pad Controls">
      <div class="metric-label">Manual Touch D-Pad (500ms Watchdog Active)</div>
      <div class="dpad-grid">
        <button type="button" class="pad-btn" data-drive="1" data-steer="-1">&#8598;<span>FWD+L</span></button>
        <button type="button" class="pad-btn" data-drive="1" data-steer="0">&#8593;<span>FORWARD</span></button>
        <button type="button" class="pad-btn" data-drive="1" data-steer="1">&#8599;<span>FWD+R</span></button>

        <button type="button" class="pad-btn" data-drive="0" data-steer="-1">&#8592;<span>LEFT</span></button>
        <button type="button" class="pad-btn stop-btn" data-drive="0" data-steer="0">STOP</button>
        <button type="button" class="pad-btn" data-drive="0" data-steer="1">&#8594;<span>RIGHT</span></button>

        <button type="button" class="pad-btn" data-drive="-1" data-steer="-1">&#8601;<span>REV+L</span></button>
        <button type="button" class="pad-btn" data-drive="-1" data-steer="0">&#8595;<span>REVERSE</span></button>
        <button type="button" class="pad-btn" data-drive="-1" data-steer="1">&#8600;<span>REV+R</span></button>
      </div>
    </section>
  </main>

  <script>
    (function() {
      var ws = null;
      var currentMode = 'standby';
      var activeDrive = 0;
      var activeSteer = 0;
      var isHoldingPad = false;

      var latestFrontCm = 400.0;
      var latestRearCm = 400.0;
      var latestState = 'INIT';

      // Target Lock state on HUD canvas
      var camActive = false;
      var targetLock = null; // { x, y, r, g, b }

      var connBadge = document.getElementById('conn-status');
      var frontDistVal = document.getElementById('front-dist-val');
      var rearDistVal = document.getElementById('rear-dist-val');
      var stateVal = document.getElementById('state-val');
      var btnAuto = document.getElementById('btn-auto');
      var btnFollow = document.getElementById('btn-follow');
      var btnManual = document.getElementById('btn-manual');
      var dpadSection = document.getElementById('dpad-section');
      var slider = document.getElementById('throttle-slider');
      var throttleReadout = document.getElementById('throttle-readout');
      var camImg = document.getElementById('cam-stream');
      var btnCamToggle = document.getElementById('btn-cam-toggle');
      var canvas = document.getElementById('hud-canvas');
      var ctx = canvas.getContext('2d');

      function syncSliderValue() {
        slider.style.setProperty('--attr-min', slider.min || 80);
        slider.style.setProperty('--attr-max', slider.max || 255);
        slider.style.setProperty('--control-value', slider.value);
        throttleReadout.textContent = slider.value + ' / 255';
        slider.setAttribute('aria-valuetext', 'PWM ' + slider.value + ' of 255');
      }

      syncSliderValue();
      slider.addEventListener('input', function() {
        syncSliderValue();
        sendPacket('TRIM:' + slider.value);
      });

      function sendPacket(payload) {
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(payload);
        }
      }

      function setMode(mode) {
        currentMode = mode;
        btnAuto.classList.toggle('active', mode === 'auto');
        btnFollow.classList.toggle('active', mode === 'follow');
        btnManual.classList.toggle('active', mode === 'manual');
        dpadSection.classList.toggle('disabled', mode !== 'manual');

        activeDrive = 0;
        activeSteer = 0;
        isHoldingPad = false;

        if (mode === 'auto') {
          sendPacket('MODE:AUTO');
        } else if (mode === 'follow') {
          sendPacket('MODE:FOLLOW');
        } else {
          sendPacket('MODE:MANUAL');
        }
      }

      btnAuto.addEventListener('click', function() { setMode('auto'); });
      btnFollow.addEventListener('click', function() { setMode('follow'); });
      btnManual.addEventListener('click', function() { setMode('manual'); });

      var CAM_STREAM_URL = 'http://192.168.4.2:81/stream';

      function connectCamera() {
        camImg.src = CAM_STREAM_URL + '?t=' + Date.now();
        camActive = true;
        btnCamToggle.textContent = 'Reconnect Cam';
      }

      btnCamToggle.addEventListener('click', function() {
        connectCamera();
      });

      // Automatically connect to the Aideepen ESP32-CAM-MB stream at 192.168.4.2:81/stream
      connectCamera();

      // Tap on HUD Canvas to Lock / Unlock a Follow-Me Target
      canvas.addEventListener('click', function(e) {
        var rect = canvas.getBoundingClientRect();
        var x = (e.clientX - rect.left) * (canvas.width / rect.width);
        var y = (e.clientY - rect.top) * (canvas.height / rect.height);

        if (targetLock && Math.hypot(x - targetLock.x, y - targetLock.y) < 38) {
          targetLock = null;
          sendPacket('TARGET:NONE');
          return;
        }

        var sampleR = 56, sampleG = 189, sampleB = 248;
        if (camActive && camImg.complete && camImg.naturalWidth > 0) {
          try {
            ctx.drawImage(camImg, 0, 0, canvas.width, canvas.height);
            var px = ctx.getImageData(Math.floor(x), Math.floor(y), 1, 1).data;
            sampleR = px[0]; sampleG = px[1]; sampleB = px[2];
          } catch (err) {}
        }

        targetLock = { x: x, y: y, r: sampleR, g: sampleG, b: sampleB };
      });

      function trackColorCentroid() {
        if (!camActive || !targetLock || !camImg.complete || camImg.naturalWidth === 0) return;
        try {
          var imgData = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
          var sumX = 0, sumY = 0, count = 0;
          var step = 4;
          for (var py = 10; py < canvas.height - 10; py += step) {
            for (var px = 10; px < canvas.width - 10; px += step) {
              var idx = (py * canvas.width + px) * 4;
              var dr = imgData[idx] - targetLock.r;
              var dg = imgData[idx + 1] - targetLock.g;
              var db = imgData[idx + 2] - targetLock.b;
              if ((dr * dr + dg * dg + db * db) < 2200) {
                sumX += px;
                sumY += py;
                count++;
              }
            }
          }
          if (count > 8) {
            targetLock.x = targetLock.x * 0.6 + (sumX / count) * 0.4;
            targetLock.y = targetLock.y * 0.6 + (sumY / count) * 0.4;
          }
        } catch (err) {}
      }

      function renderHud() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        if (camActive && camImg.complete && camImg.naturalWidth > 0) {
          try {
            ctx.drawImage(camImg, 0, 0, canvas.width, canvas.height);
            trackColorCentroid();
          } catch (err) {}
        } else {
          // Radar grid background when no camera is connected
          ctx.strokeStyle = '#1e293b';
          ctx.lineWidth = 1;
          for (var gx = 40; gx < canvas.width; gx += 40) {
            ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, canvas.height); ctx.stroke();
          }
          for (var gy = 38; gy < canvas.height; gy += 38) {
            ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(canvas.width, gy); ctx.stroke();
          }
        }

        // Draw Front Sonar Obstacle Highlight Box
        var frontColor = '#22c55e';
        var frontTag = 'CLEAR PATH';
        if (latestFrontCm < 28) {
          frontColor = '#ef4444';
          frontTag = 'BLOCKED (<28cm)';
        } else if (latestFrontCm < 65) {
          frontColor = '#f59e0b';
          frontTag = 'STEER AROUND (' + latestFrontCm.toFixed(0) + 'cm)';
        }

        var boxW = Math.max(70, Math.min(260, (120 - Math.min(latestFrontCm, 100)) * 2.2));
        var boxH = Math.max(36, Math.min(110, (120 - Math.min(latestFrontCm, 100)) * 0.95));
        var bx = (canvas.width - boxW) / 2;
        var by = 22;

        ctx.strokeStyle = frontColor;
        ctx.lineWidth = 2;
        ctx.strokeRect(bx, by, boxW, boxH);
        ctx.fillStyle = 'rgba(15, 23, 42, 0.78)';
        ctx.fillRect(bx, by - 16, 155, 16);
        ctx.fillStyle = frontColor;
        ctx.font = 'bold 10px sans-serif';
        ctx.fillText('FRONT: ' + frontTag, bx + 4, by - 4);

        // Draw Rear Proximity Bar at bottom of HUD
        var rearColor = (latestRearCm < 15) ? '#ef4444' : ((latestRearCm < 35) ? '#f59e0b' : '#38bdf8');
        ctx.fillStyle = 'rgba(15, 23, 42, 0.8)';
        ctx.fillRect(110, canvas.height - 22, 180, 18);
        ctx.strokeStyle = rearColor;
        ctx.strokeRect(110, canvas.height - 22, 180, 18);
        ctx.fillStyle = rearColor;
        ctx.fillText('REAR BUMPER: ' + latestRearCm.toFixed(1) + ' cm', 135, canvas.height - 9);

        // Draw Locked Target Bounding Box & Steering Vector
        if (targetLock) {
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2.5;
          ctx.strokeRect(targetLock.x - 32, targetLock.y - 32, 64, 64);
          ctx.beginPath();
          ctx.moveTo(targetLock.x - 10, targetLock.y);
          ctx.lineTo(targetLock.x + 10, targetLock.y);
          ctx.moveTo(targetLock.x, targetLock.y - 10);
          ctx.lineTo(targetLock.x, targetLock.y + 10);
          ctx.stroke();

          var steerCmd = 0;
          if (targetLock.x < canvas.width * 0.38) steerCmd = -1;
          else if (targetLock.x > canvas.width * 0.62) steerCmd = 1;

          ctx.fillStyle = '#38bdf8';
          ctx.fillText(
            'LOCKED [' + (steerCmd < 0 ? 'LEFT' : (steerCmd > 0 ? 'RIGHT' : 'CENTER')) + ']',
            targetLock.x - 32,
            targetLock.y - 37
          );
        }

        requestAnimationFrame(renderHud);
      }
      requestAnimationFrame(renderHud);

      var padButtons = document.querySelectorAll('.pad-btn');
      for (var i = 0; i < padButtons.length; i++) {
        (function(btn) {
          var d = parseInt(btn.getAttribute('data-drive'), 10);
          var s = parseInt(btn.getAttribute('data-steer'), 10);

          function engage(e) {
            if (e.cancelable) e.preventDefault();
            if (currentMode !== 'manual') return;
            activeDrive = d;
            activeSteer = s;
            isHoldingPad = (d !== 0 || s !== 0);
            btn.classList.add('pressed');
            sendPacket('CMD:' + activeDrive + ':' + activeSteer);
          }

          function release(e) {
            if (e && e.cancelable) e.preventDefault();
            btn.classList.remove('pressed');
            if (currentMode !== 'manual') return;
            activeDrive = 0;
            activeSteer = 0;
            isHoldingPad = false;
            sendPacket('CMD:0:0');
          }

          btn.addEventListener('touchstart', engage, { passive: false });
          btn.addEventListener('touchend', release, { passive: false });
          btn.addEventListener('touchcancel', release, { passive: false });
          btn.addEventListener('mousedown', engage);
          btn.addEventListener('mouseup', release);
          btn.addEventListener('mouseleave', function() {
            if (btn.classList.contains('pressed')) release();
          });
        })(padButtons[i]);
      }

      // Send 150ms heartbeat & target lock steering vector
      setInterval(function() {
        if (currentMode === 'manual') {
          if (isHoldingPad) {
            sendPacket('CMD:' + activeDrive + ':' + activeSteer);
          } else {
            sendPacket('HB');
          }
        } else if (currentMode === 'follow' && targetLock) {
          var steerCmd = 0;
          if (targetLock.x < canvas.width * 0.38) steerCmd = -1;
          else if (targetLock.x > canvas.width * 0.62) steerCmd = 1;
          sendPacket('TARGET:' + steerCmd);
        }
      }, 150);

      function colorDistanceElement(el, cm) {
        el.textContent = cm.toFixed(1) + ' cm';
        if (cm < 28) {
          el.style.color = '#ef4444';
        } else if (cm < 65) {
          el.style.color = '#f59e0b';
        } else {
          el.style.color = '#38bdf8';
        }
      }

      function connectWs() {
        var proto = (window.location.protocol === 'https:') ? 'wss://' : 'ws://';
        ws = new WebSocket(proto + window.location.host + '/ws');

        ws.onopen = function() {
          connBadge.textContent = 'ONLINE';
          connBadge.classList.add('online');
        };

        ws.onclose = function() {
          connBadge.textContent = 'OFFLINE';
          connBadge.classList.remove('online');
          setTimeout(connectWs, 1000);
        };

        ws.onmessage = function(evt) {
          try {
            var data = JSON.parse(evt.data);
            if (typeof data.front === 'number') {
              latestFrontCm = data.front;
              colorDistanceElement(frontDistVal, data.front);
            }
            if (typeof data.rear === 'number') {
              latestRearCm = data.rear;
              colorDistanceElement(rearDistVal, data.rear);
            }
            if (data.state) {
              latestState = data.state;
              stateVal.textContent = data.state;
            }
            if (typeof data.mode === 'number') {
              var serverMode = (data.mode === 1) ? 'auto' : ((data.mode === 2) ? 'follow' : ((data.mode === 3) ? 'manual' : 'standby'));
              if (serverMode !== currentMode) {
                currentMode = serverMode;
                btnAuto.classList.toggle('active', currentMode === 'auto');
                btnFollow.classList.toggle('active', currentMode === 'follow');
                btnManual.classList.toggle('active', currentMode === 'manual');
                dpadSection.classList.toggle('disabled', currentMode !== 'manual');
              }
            }
          } catch (err) {}
        };
      }

      connectWs();
    })();
  </script>
</body>
</html>
)rawliteral";

} // namespace

WebInterface::WebInterface()
    : _server(80), _ws("/ws") {}

void WebInterface::begin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASS);

    _ws.onEvent([this](AsyncWebSocket* server,
                       AsyncWebSocketClient* client,
                       AwsEventType type,
                       void* arg,
                       uint8_t* data,
                       size_t len) {
        this->onWsEvent(server, client, type, arg, data, len);
    });

    _server.addHandler(&_ws);

    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send_P(200, "text/html", INDEX_HTML);
    });

    _server.begin();
    _lastHeartbeatMs = millis();
}

void WebInterface::onWsEvent(AsyncWebSocket* server,
                             AsyncWebSocketClient* client,
                             AwsEventType type,
                             void* arg,
                             uint8_t* data,
                             size_t len) {
    (void)server;
    (void)client;

    if (type == WS_EVT_CONNECT) {
        _lastHeartbeatMs = millis();
    } else if (type == WS_EVT_DISCONNECT) {
        clearManualCommand();
        _followTargetLocked = false;
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = reinterpret_cast<AwsFrameInfo*>(arg);
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            handleMessage(data, len);
        }
    }
}

void WebInterface::handleMessage(uint8_t* data, size_t len) {
    if (len == 0 || len >= 64) {
        return;
    }

    char buf[64];
    memcpy(buf, data, len);
    buf[len] = '\0';

    _lastHeartbeatMs = millis();

    if (strcmp(buf, "HB") == 0) {
        return;
    }

    // Any explicit command from the web UI cancels the 30-second boot standby timer
    _userInteracted = true;

    if (strcmp(buf, "MODE:AUTO") == 0) {
        clearManualCommand();
        _mode = RoverMode::AUTONOMOUS;
        return;
    }

    if (strcmp(buf, "MODE:FOLLOW") == 0) {
        clearManualCommand();
        _mode = RoverMode::FOLLOW;
        return;
    }

    if (strcmp(buf, "MODE:MANUAL") == 0) {
        clearManualCommand();
        _mode = RoverMode::MANUAL;
        return;
    }

    if (strncmp(buf, "TARGET:", 7) == 0) {
        if (strcmp(buf + 7, "NONE") == 0) {
            _followTargetLocked = false;
            _followTargetSteer = 0;
        } else {
            int steerVal = atoi(buf + 7);
            _followTargetSteer = (steerVal > 0) ? 1 : ((steerVal < 0) ? -1 : 0);
            _followTargetLocked = true;
            _lastFollowUpdateMs = millis();
        }
        return;
    }

    if (strncmp(buf, "TRIM:", 5) == 0) {
        int val = atoi(buf + 5);
        if (val < 80) val = 80;
        if (val > 255) val = 255;
        _throttleTrim = static_cast<uint8_t>(val);
        return;
    }

    if (strncmp(buf, "CMD:", 4) == 0) {
        char* firstColon = buf + 3;
        char* secondColon = strchr(firstColon + 1, ':');
        if (secondColon != nullptr) {
            *secondColon = '\0';
            int driveVal = atoi(firstColon + 1);
            int steerVal = atoi(secondColon + 1);

            _manualCmd.drive = (driveVal > 0) ? 1 : ((driveVal < 0) ? -1 : 0);
            _manualCmd.steer = (steerVal > 0) ? 1 : ((steerVal < 0) ? -1 : 0);
        }
    }
}

int8_t WebInterface::getFollowTargetSteer() const {
    if (!_followTargetLocked || (millis() - _lastFollowUpdateMs > 1000)) {
        return 0;
    }
    return _followTargetSteer;
}

bool WebInterface::hasActiveFollowTarget() const {
    return _followTargetLocked && (millis() - _lastFollowUpdateMs <= 1000);
}

bool WebInterface::isManualWatchdogExpired() const {
    return (millis() - _lastHeartbeatMs) > WATCHDOG_TIMEOUT_MS;
}

void WebInterface::clearManualCommand() {
    _manualCmd.drive = 0;
    _manualCmd.steer = 0;
}

void WebInterface::broadcastTelemetry(float frontDistanceCm, float rearDistanceCm, const char* stateLabel, bool watchdogTripped) {
    if (_ws.count() == 0) {
        return;
    }

    char json[180];
    snprintf(json,
             sizeof(json),
             "{\"front\":%.1f,\"rear\":%.1f,\"state\":\"%s\",\"wd\":%s,\"mode\":%u}",
             frontDistanceCm,
             rearDistanceCm,
             stateLabel,
             watchdogTripped ? "true" : "false",
             static_cast<unsigned>(_mode));
    _ws.textAll(json);
}

void WebInterface::update(float frontDistanceCm, float rearDistanceCm, const char* stateLabel, bool watchdogTripped) {
    _ws.cleanupClients();

    const uint32_t nowMs = millis();
    if (nowMs - _lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        _lastTelemetryMs = nowMs;
        broadcastTelemetry(frontDistanceCm, rearDistanceCm, stateLabel, watchdogTripped);
    }
}
