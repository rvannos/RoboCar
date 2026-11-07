#include "web_interface.h"
#include "img_converters.h"
#include <math.h>
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
  <title>RC Rover Heat Map &amp; Control</title>
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

    /* Live Heat Map / Camera Viewport */
    .hud-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
      gap: 6px;
    }

    .hud-actions {
      display: flex;
      gap: 6px;
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
      height: 224px;
      background: #060a12;
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

    .sector-bar {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 6px;
      margin-top: 8px;
    }

    .sector-pill {
      background: #0f172a;
      border-radius: 6px;
      padding: 4px 6px;
      text-align: center;
      font-size: 0.7rem;
      color: var(--text-muted);
      font-variant-numeric: tabular-nums;
    }

    .sector-pill strong {
      color: var(--text-main);
      display: block;
      font-size: 0.8rem;
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
      --control-value: attr(value type(<number>), 135);
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

    /* Collapsible Home Wi-Fi & Wireless OTA Setup Card */
    .wifi-details summary {
      display: flex;
      justify-content: space-between;
      align-items: center;
      cursor: pointer;
      font-size: 0.85rem;
      font-weight: 600;
      list-style: none;
    }

    .wifi-details summary::-webkit-details-marker {
      display: none;
    }

    .wifi-details summary::before {
      content: "▸ ";
      color: var(--brand-color);
      margin-right: 4px;
    }

    .wifi-details[open] summary::before {
      content: "▾ ";
    }

    .wifi-form {
      margin-top: 12px;
      padding-top: 12px;
      border-top: 1px solid #334155;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }

    .field-group {
      display: flex;
      flex-direction: column;
      gap: 4px;
    }

    .field-group label {
      font-size: 0.78rem;
      font-weight: 600;
      color: var(--text-muted);
    }

    .text-input {
      width: 100%;
      min-height: 48px;
      padding: 10px 12px;
      font-size: 1rem;
      color: var(--text-main);
      background: #0f172a;
      border: 1px solid #475569;
      border-radius: 8px;
      user-select: text;
      -webkit-user-select: text;
    }

    .text-input:focus-visible {
      outline: 2px solid var(--brand-color);
      outline-offset: 2px;
    }

    .pw-row {
      display: flex;
      gap: 8px;
      align-items: stretch;
    }

    .submit-btn {
      border: 0;
      border-radius: 8px;
      min-height: 48px;
      font-size: 0.9rem;
      font-weight: 700;
      background: var(--brand-color);
      color: #0f172a;
      cursor: pointer;
    }

    .submit-btn:disabled {
      opacity: 0.5;
      cursor: default;
    }

    .submit-btn:focus-visible {
      outline: 2px solid var(--brand-color);
      outline-offset: 2px;
    }
  </style>
</head>
<body>
  <main class="container">
    <header class="header">
      <h1>RC-ROVER S3</h1>
      <span id="conn-status" class="conn-badge">CONNECTING...</span>
    </header>

    <section class="card" aria-label="2D Sonar Heat Map &amp; HUD">
      <div class="hud-header">
        <span class="metric-label" style="margin:0;">2D Sonar Obstacle Heat Map</span>
        <div class="hud-actions">
          <button type="button" id="btn-clear-map" class="cam-btn">Clear Map</button>
          <button type="button" id="btn-cam-rot" class="cam-btn" title="Rotate Camera 90°">Rot 90&deg;</button>
          <button type="button" id="btn-cam-toggle" class="cam-btn">Cam Overlay</button>
        </div>
      </div>
      <div class="hud-viewport">
        <img id="cam-stream" crossorigin="anonymous" alt="ESP32-CAM Live Stream">
        <canvas id="hud-canvas" width="400" height="224"></canvas>
      </div>
      <div class="sector-bar">
        <div class="sector-pill">LEFT SECTOR<strong id="sec-l-val">-- cm</strong></div>
        <div class="sector-pill">CENTER SECTOR<strong id="sec-c-val">-- cm</strong></div>
        <div class="sector-pill">RIGHT SECTOR<strong id="sec-r-val">-- cm</strong></div>
      </div>

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
        <span id="throttle-readout" class="slider-value">135 / 255</span>
      </div>
      <input
        type="range"
        id="throttle-slider"
        min="80"
        max="255"
        step="5"
        value="135"
        aria-valuetext="PWM 135 of 255"
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

    <details class="card wifi-details">
      <summary>
        <span>Home Wi-Fi &amp; Wireless OTA</span>
        <span id="wifi-sta-badge" class="conn-badge">AP ONLY</span>
      </summary>
      <form id="wifi-form" class="wifi-form" action="/wifi" method="POST">
        <div class="field-group">
          <label for="wifi-ssid">Wi-Fi Network Name (SSID)</label>
          <input
            type="text"
            id="wifi-ssid"
            name="ssid"
            class="text-input"
            required
            maxlength="32"
            autocorrect="off"
            autocapitalize="off"
            spellcheck="false"
            enterkeyhint="next"
          >
        </div>
        <div class="field-group">
          <label for="wifi-pass">Wi-Fi Password</label>
          <div class="pw-row">
            <input
              type="password"
              id="wifi-pass"
              name="pass"
              class="text-input"
              maxlength="64"
              autocomplete="current-password"
              enterkeyhint="done"
            >
            <button type="button" id="btn-toggle-pw" class="cam-btn" aria-pressed="false" aria-label="Show password">Show</button>
          </div>
        </div>
        <button type="submit" id="btn-wifi-save" class="submit-btn">Save &amp; Connect Wi-Fi</button>
        <div id="wifi-status-msg" class="sub-hint" aria-live="polite">Keeps RC-Rover hotspot active while joining home Wi-Fi for wireless OTA updates.</div>
      </form>
    </details>
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
      var latestDriveDir = 0;
      var latestSteerDir = 0;
      var latestGoalSteer = 0;
      var latestHeadingDeg = 0;
      var latestState = 'INIT';

      var camActive = false;
      var camFrameReady = false;
      var camRotRad = Math.PI; // 180 deg default (90 deg + 90 deg) so mounted camera is right-side up
      var camDisplayCanvas = document.createElement('canvas');
      camDisplayCanvas.width = 400;
      camDisplayCanvas.height = 224;
      var camDisplayCtx = camDisplayCanvas.getContext('2d');
      var targetLock = null;
      var visionCols = 16;
      var visionDepthsCm = new Float32Array(16);
      var offCanvas = document.createElement('canvas');
      offCanvas.width = 80;
      offCanvas.height = 60;
      var offCtx = offCanvas.getContext('2d', { willReadFrequently: true });
      var lastVisionSendMs = 0;

      function drawRotatedCam(targetCtx, w, h) {
        targetCtx.save();
        targetCtx.translate(w / 2, h / 2);
        targetCtx.rotate(camRotRad);
        if (Math.abs(Math.sin(camRotRad)) > 0.5) {
          targetCtx.drawImage(camImg, -h / 2, -w / 2, h, w);
        } else {
          targetCtx.drawImage(camImg, -w / 2, -h / 2, w, h);
        }
        targetCtx.restore();
      }

      // 2D Top-Down Occupancy Heat Map Grid (50 cols x 28 rows, 8px per cell, 1 cell = 4cm)
      var GRID_COLS = 50;
      var GRID_ROWS = 28;
      var CELL_PX = 8;
      var CM_PER_CELL = 4.0;
      var ROVER_GX = 25.0;
      var ROVER_GY = 14.0;
      var heatGrid = new Float32Array(GRID_COLS * GRID_ROWS);
      var tempGrid = new Float32Array(GRID_COLS * GRID_ROWS);
      var freeGrid = new Float32Array(GRID_COLS * GRID_ROWS);
      var tempFree = new Float32Array(GRID_COLS * GRID_ROWS);

      var connBadge = document.getElementById('conn-status');
      var frontDistVal = document.getElementById('front-dist-val');
      var rearDistVal = document.getElementById('rear-dist-val');
      var secLVal = document.getElementById('sec-l-val');
      var secCVal = document.getElementById('sec-c-val');
      var secRVal = document.getElementById('sec-r-val');
      var stateVal = document.getElementById('state-val');
      var btnAuto = document.getElementById('btn-auto');
      var btnFollow = document.getElementById('btn-follow');
      var btnManual = document.getElementById('btn-manual');
      var dpadSection = document.getElementById('dpad-section');
      var slider = document.getElementById('throttle-slider');
      var throttleReadout = document.getElementById('throttle-readout');
      var camImg = document.getElementById('cam-stream');
      var btnCamToggle = document.getElementById('btn-cam-toggle');
      var btnCamRot = document.getElementById('btn-cam-rot');
      var btnClearMap = document.getElementById('btn-clear-map');
      var wifiForm = document.getElementById('wifi-form');
      var wifiSsidInput = document.getElementById('wifi-ssid');
      var wifiPassInput = document.getElementById('wifi-pass');
      var btnTogglePw = document.getElementById('btn-toggle-pw');
      var btnWifiSave = document.getElementById('btn-wifi-save');
      var wifiStaBadge = document.getElementById('wifi-sta-badge');
      var wifiStatusMsg = document.getElementById('wifi-status-msg');
      var canvas = document.getElementById('hud-canvas');
      var ctx = canvas.getContext('2d');

      btnTogglePw.addEventListener('click', function() {
        var showing = wifiPassInput.type === 'text';
        wifiPassInput.type = showing ? 'password' : 'text';
        btnTogglePw.setAttribute('aria-pressed', showing ? 'false' : 'true');
        btnTogglePw.textContent = showing ? 'Show' : 'Hide';
      });

      wifiForm.addEventListener('submit', function(e) {
        e.preventDefault();
        var ssid = wifiSsidInput.value.trim();
        var pass = wifiPassInput.value;
        if (!ssid) return;
        btnWifiSave.disabled = true;
        wifiStatusMsg.textContent = 'Saving & connecting to ' + ssid + '...';
        sendPacket('WIFI:' + ssid + '\t' + pass);
        setTimeout(function() { btnWifiSave.disabled = false; }, 2500);
      });

      function clearHeatMap() {
        for (var i = 0; i < heatGrid.length; i++) {
          heatGrid[i] = 0;
          freeGrid[i] = 0;
        }
      }

      btnClearMap.addEventListener('click', clearHeatMap);

      // Advect (shift/rotate) the heat map & free-space corridor map when the rover moves or turns
      function stepHeatMapPhysics(driveDir, steerDir, frontCm, rearCm) {
        var dy = driveDir * 0.42; // Shift obstacles opposite to car motion
        var dTheta = -steerDir * (driveDir !== 0 ? 0.055 : 0.015);
        var cosT = Math.cos(dTheta);
        var sinT = Math.sin(dTheta);

        for (var r = 0; r < GRID_ROWS; r++) {
          for (var c = 0; c < GRID_COLS; c++) {
            var rx = c - ROVER_GX;
            var ry = r - ROVER_GY - dy;
            var srcX = Math.round(ROVER_GX + (rx * cosT - ry * sinT));
            var srcY = Math.round(ROVER_GY + (rx * sinT + ry * cosT));
            var val = 0;
            var fVal = 0;
            if (srcX >= 0 && srcX < GRID_COLS && srcY >= 0 && srcY < GRID_ROWS) {
              var sIdx = srcY * GRID_COLS + srcX;
              val = heatGrid[sIdx] * 0.99; // Persistent obstacle memory
              fVal = freeGrid[sIdx] * 0.992; // Persistent explored free-corridor memory
            }
            tempGrid[r * GRID_COLS + c] = val;
            tempFree[r * GRID_COLS + c] = fVal;
          }
        }
        heatGrid.set(tempGrid);
        freeGrid.set(tempFree);

        // Splat Front Sonar Beam (-90 deg = Up on canvas; steer offsets beam by +/- 26 deg)
        var frontBaseAngle = -Math.PI / 2 + (steerDir * 0.45);
        splatSonarCone(frontBaseAngle, frontCm, 2.0);

        // If Camera + Sonar Fusion is active, splat all 16 optical floor-horizon rays across a 60-deg FOV
        if (camActive && camFrameReady) {
          runVisionSonarFusion(frontCm);
        }

        // Splat Rear Sonar Beam (+90 deg = Down on canvas)
        var rearBaseAngle = Math.PI / 2;
        splatSonarCone(rearBaseAngle, rearCm, 2.0);
      }

      // Camera + Sonar Sensor Fusion:
      // 1. Samples the floor color/luminance patch directly ahead of the front bumper
      // 2. Scans upward in 16 vertical columns across the camera's 60-deg horizontal FOV to find the obstacle base
      // 3. Calibrates the center optical horizon row against the Front Sonar's exact centimeter distance
      // 4. Converts all 16 columns into real centimeter distances, splats them into the 2D Heat Map,
      //    and feeds Left/Right visual clearances back to the ESP32-S3 steering controller!
      function runVisionSonarFusion(frontSonarCm) {
        try {
          offCtx.drawImage(camDisplayCanvas, 0, 0, 80, 60);
          var img = offCtx.getImageData(0, 0, 80, 60).data;

          // Sample reference floor patch at bottom-center (x: 32..48, y: 50..57)
          var refR = 0, refG = 0, refB = 0, count = 0;
          for (var fy = 50; fy < 58; fy += 2) {
            for (var fx = 32; fx < 48; fx += 2) {
              var p = (fy * 80 + fx) * 4;
              refR += img[p]; refG += img[p + 1]; refB += img[p + 2];
              count++;
            }
          }
          refR /= count; refG /= count; refB /= count;

          var rawDistNorm = new Float32Array(visionCols);
          for (var c = 0; c < visionCols; c++) {
            var cx = Math.floor((c + 0.5) * (80 / visionCols));
            var hitY = 12; // Horizon row
            for (var y = 55; y >= 12; y--) {
              var idx = (y * 80 + cx) * 4;
              var dr = Math.abs(img[idx] - refR);
              var dg = Math.abs(img[idx + 1] - refG);
              var db = Math.abs(img[idx + 2] - refB);
              if (dr + dg + db > 88) {
                hitY = y;
                break;
              }
            }
            // Perspective ground-plane approximation: higher up on image = farther away
            var norm = Math.max(0.08, (56 - hitY) / 44.0);
            rawDistNorm[c] = norm;
          }

          // Calibrate optical scale using the center columns (7 & 8) + true Front Sonar distance
          var centerNorm = (rawDistNorm[7] + rawDistNorm[8]) * 0.5;
          var clampedSonar = Math.min(Math.max(frontSonarCm, 12.0), 140.0);
          var scaleCm = clampedSonar / Math.max(centerNorm, 0.15);

          var leftSum = 0, rightSum = 0;
          for (var i = 0; i < visionCols; i++) {
            var colCm = Math.min(Math.max(rawDistNorm[i] * scaleCm, 10.0), 150.0);
            // Blend center columns tightly with acoustic truth
            if (i >= 6 && i <= 9) {
              colCm = colCm * 0.35 + clampedSonar * 0.65;
            }
            visionDepthsCm[i] = colCm;
            if (i < 6) leftSum += colCm;
            if (i > 9) rightSum += colCm;

            // Splat narrow 3.5-deg optical-acoustic ray into the 2D Heat Map across -30 deg to +30 deg
            var rayAngle = -Math.PI / 2 + ((i - 7.5) / 7.5) * 0.52;
            splatVisionRay(rayAngle, colCm, 2.0);
          }

          var now = Date.now();
          if (now - lastVisionSendMs > 180) {
            lastVisionSendMs = now;
            var visLeftCm = Math.round(leftSum / 6.0);
            var visRightCm = Math.round(rightSum / 6.0);
            sendPacket('VIS:' + visLeftCm + ':' + visRightCm);
          }
        } catch (err) {}
      }

      function splatVisionRay(angleRad, distCm, bumperOffsetCells) {
        var ca = Math.cos(angleRad);
        var sa = Math.sin(angleRad);
        var maxCells = Math.min(distCm / CM_PER_CELL, 26);
        for (var d = bumperOffsetCells; d < maxCells - 1.0; d += 1.0) {
          var gx = Math.round(ROVER_GX + ca * d);
          var gy = Math.round(ROVER_GY + sa * d);
          if (gx >= 0 && gx < GRID_COLS && gy >= 0 && gy < GRID_ROWS) {
            heatGrid[gy * GRID_COLS + gx] *= 0.85;
          }
        }
        if (distCm < 110.0) {
          var hitR = bumperOffsetCells + (distCm / CM_PER_CELL);
          var hx = Math.round(ROVER_GX + ca * hitR);
          var hy = Math.round(ROVER_GY + sa * hitR);
          if (hx >= 0 && hx < GRID_COLS && hy >= 0 && hy < GRID_ROWS) {
            heatGrid[hy * GRID_COLS + hx] = Math.min(1.0, heatGrid[hy * GRID_COLS + hx] + 0.22);
          }
        }
      }

      function splatSonarCone(centerAngleRad, distCm, bumperOffsetCells) {
        var halfCone = 0.20; // ~23 deg total acoustic cone
        var maxCells = Math.min(distCm / CM_PER_CELL, 28);

        // 1. Carve out free space along the acoustic beam inside the cone & mark explored green corridor
        for (var a = -halfCone; a <= halfCone; a += 0.06) {
          var ang = centerAngleRad + a;
          var ca = Math.cos(ang);
          var sa = Math.sin(ang);
          for (var d = bumperOffsetCells; d < maxCells - 1.2; d += 0.9) {
            var gx = Math.round(ROVER_GX + ca * d);
            var gy = Math.round(ROVER_GY + sa * d);
            if (gx >= 0 && gx < GRID_COLS && gy >= 0 && gy < GRID_ROWS) {
              var cIdx = gy * GRID_COLS + gx;
              heatGrid[cIdx] *= 0.68;
              freeGrid[cIdx] = Math.min(1.0, freeGrid[cIdx] + 0.18);
            }
          }
        }

        // 2. Deposit thermal obstacle energy along the echo arc if within map range (< 110cm)
        if (distCm < 110.0 && distCm >= 2.0) {
          var hitRadiusCells = bumperOffsetCells + (distCm / CM_PER_CELL);
          var intensityBoost = (distCm < 30) ? 0.55 : ((distCm < 65) ? 0.40 : 0.28);
          for (var a2 = -halfCone; a2 <= halfCone; a2 += 0.04) {
            var ang2 = centerAngleRad + a2;
            var hx = ROVER_GX + Math.cos(ang2) * hitRadiusCells;
            var hy = ROVER_GY + Math.sin(ang2) * hitRadiusCells;

            // Gaussian 3x3 thermal kernel splat
            for (var ky = -1; ky <= 1; ky++) {
              for (var kx = -1; kx <= 1; kx++) {
                var ix = Math.round(hx) + kx;
                var iy = Math.round(hy) + ky;
                if (ix >= 0 && ix < GRID_COLS && iy >= 0 && iy < GRID_ROWS) {
                  var weight = (kx === 0 && ky === 0) ? 1.0 : 0.45;
                  var idx = iy * GRID_COLS + ix;
                  heatGrid[idx] = Math.min(1.0, heatGrid[idx] + intensityBoost * weight);
                }
              }
            }
          }
        }
      }

      function heatToColor(v) {
        // Thermal palette: Cyan/Blue (0.1) -> Green (0.35) -> Yellow (0.65) -> Orange/Red (1.0)
        if (v < 0.35) {
          var t = v / 0.35;
          return 'rgba(' + Math.round(20 * t) + ',' + Math.round(140 + 80 * t) + ',' + Math.round(245 - 120 * t) + ',' + (0.25 + t * 0.45).toFixed(2) + ')';
        } else if (v < 0.7) {
          var t2 = (v - 0.35) / 0.35;
          return 'rgba(' + Math.round(20 + 225 * t2) + ',' + Math.round(220 - 40 * t2) + ',' + Math.round(125 - 110 * t2) + ',' + (0.7 + t2 * 0.15).toFixed(2) + ')';
        } else {
          var t3 = (v - 0.7) / 0.3;
          return 'rgba(248,' + Math.round(180 - 135 * t3) + ',' + Math.round(25 + 30 * t3) + ',0.92)';
        }
      }

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

      var CAM_DIRECT_URL = 'http://192.168.4.2:81/stream';
      var useRelayCam = (window.location.hostname !== '192.168.4.1');
      var camRelayTimer = null;
      var camWatchdogTimer = null;

      function fetchNextRelayFrame() {
        if (!camActive || !useRelayCam) return;
        clearTimeout(camWatchdogTimer);
        camWatchdogTimer = setTimeout(function() {
          if (camActive && useRelayCam) {
            fetchNextRelayFrame();
          }
        }, 1800);
        camImg.src = '/cam.jpg?t=' + Date.now();
      }

      function scheduleNextRelayFrame(delayMs) {
        if (!camActive || !useRelayCam) return;
        clearTimeout(camRelayTimer);
        camRelayTimer = setTimeout(fetchNextRelayFrame, delayMs);
      }

      camImg.addEventListener('load', function() {
        clearTimeout(camWatchdogTimer);
        if (camImg.naturalWidth > 0) {
          drawRotatedCam(camDisplayCtx, camDisplayCanvas.width, camDisplayCanvas.height);
          camFrameReady = true;
        }
        if (camActive && useRelayCam) {
          scheduleNextRelayFrame(110);
        }
      });

      camImg.addEventListener('error', function() {
        clearTimeout(camWatchdogTimer);
        if (camActive && useRelayCam) {
          scheduleNextRelayFrame(250);
        }
      });

      btnCamRot.addEventListener('click', function() {
        camRotRad = (camRotRad + Math.PI / 2) % (Math.PI * 2);
        if (camImg.complete && camImg.naturalWidth > 0) {
          drawRotatedCam(camDisplayCtx, camDisplayCanvas.width, camDisplayCanvas.height);
          camFrameReady = true;
        }
      });

      btnCamToggle.addEventListener('click', function() {
        camActive = !camActive;
        clearTimeout(camRelayTimer);
        clearTimeout(camWatchdogTimer);
        if (camActive) {
          if (useRelayCam) {
            fetchNextRelayFrame();
          } else {
            camImg.src = CAM_DIRECT_URL + '?t=' + Date.now();
          }
          btnCamToggle.textContent = 'Hide Cam';
        } else {
          camFrameReady = false;
          camImg.src = '';
          btnCamToggle.textContent = 'Cam Overlay';
        }
      });

      canvas.addEventListener('click', function(e) {
        var rect = canvas.getBoundingClientRect();
        var x = (e.clientX - rect.left) * (canvas.width / rect.width);
        var y = (e.clientY - rect.top) * (canvas.height / rect.height);

        if (targetLock && Math.hypot(x - targetLock.x, y - targetLock.y) < 38) {
          targetLock = null;
          sendPacket('TARGET:NONE');
          return;
        }
        targetLock = { x: x, y: y, r: 56, g: 189, b: 248 };
      });

      function renderHud() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        if (camActive && !useRelayCam && camImg.complete && camImg.naturalWidth > 0) {
          drawRotatedCam(camDisplayCtx, camDisplayCanvas.width, camDisplayCanvas.height);
          camFrameReady = true;
        }

        if (camActive && camFrameReady) {
          try {
            ctx.globalAlpha = 0.52;
            ctx.drawImage(camDisplayCanvas, 0, 0, canvas.width, canvas.height);
            ctx.globalAlpha = 1.0;

            // Draw Fused Drivable Floor Horizon Polygon across the 16 optical-sonar columns
            ctx.fillStyle = 'rgba(34, 197, 94, 0.18)';
            ctx.strokeStyle = 'rgba(56, 189, 248, 0.85)';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(0, canvas.height);
            for (var vc = 0; vc < visionCols; vc++) {
              var vx = (vc + 0.5) * (canvas.width / visionCols);
              var vy = canvas.height - Math.min(1.0, visionDepthsCm[vc] / 140.0) * (canvas.height * 0.72);
              ctx.lineTo(vx, vy);
            }
            ctx.lineTo(canvas.width, canvas.height);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
          } catch (err) {}
        }

        // 1. Draw Polar Range Rings (30cm and 65cm thresholds) & Radar Grid
        var cx = ROVER_GX * CELL_PX;
        var cy = ROVER_GY * CELL_PX;

        ctx.strokeStyle = 'rgba(51, 65, 85, 0.45)';
        ctx.lineWidth = 1;
        for (var gx = 0; gx <= canvas.width; gx += 32) {
          ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, canvas.height); ctx.stroke();
        }
        for (var gy = 0; gy <= canvas.height; gy += 32) {
          ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(canvas.width, gy); ctx.stroke();
        }

        // 28cm Reverse Ring & 65cm Steer-Around Ring
        ctx.setLineDash([4, 4]);
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.35)';
        ctx.beginPath(); ctx.arc(cx, cy, (28 / CM_PER_CELL) * CELL_PX, 0, Math.PI * 2); ctx.stroke();
        ctx.strokeStyle = 'rgba(245, 158, 11, 0.35)';
        ctx.beginPath(); ctx.arc(cx, cy, (65 / CM_PER_CELL) * CELL_PX, 0, Math.PI * 2); ctx.stroke();
        ctx.setLineDash([]);

        // 2. Draw Active Acoustic Sonar Beam Cones
        var frontAng = -Math.PI / 2 + (latestSteerDir * 0.45);
        var frontLenPx = Math.min(latestFrontCm / CM_PER_CELL, 14) * CELL_PX;
        ctx.fillStyle = (latestFrontCm < 28) ? 'rgba(239, 68, 68, 0.16)' : 'rgba(56, 189, 248, 0.12)';
        ctx.beginPath();
        ctx.moveTo(cx, cy - 12);
        ctx.arc(cx, cy - 12, frontLenPx, frontAng - 0.22, frontAng + 0.22);
        ctx.closePath();
        ctx.fill();

        var rearLenPx = Math.min(latestRearCm / CM_PER_CELL, 12) * CELL_PX;
        ctx.fillStyle = (latestRearCm < 15) ? 'rgba(239, 68, 68, 0.18)' : 'rgba(168, 85, 247, 0.12)';
        ctx.beginPath();
        ctx.moveTo(cx, cy + 12);
        ctx.arc(cx, cy + 12, rearLenPx, Math.PI / 2 - 0.22, Math.PI / 2 + 0.22);
        ctx.closePath();
        ctx.fill();

        // 3. Render 2D Explored Free-Space Corridors (Green) & Obstacle Heat Map Cells (Cyan->Yellow->Red)
        for (var r = 0; r < GRID_ROWS; r++) {
          for (var c = 0; c < GRID_COLS; c++) {
            var cellIdx = r * GRID_COLS + c;
            var val = heatGrid[cellIdx];
            var fVal = freeGrid[cellIdx];
            if (val > 0.06) {
              ctx.fillStyle = heatToColor(val);
              ctx.beginPath();
              ctx.roundRect(c * CELL_PX + 1, r * CELL_PX + 1, CELL_PX - 1, CELL_PX - 1, 3);
              ctx.fill();
            } else if (fVal > 0.12) {
              ctx.fillStyle = 'rgba(34, 197, 94, ' + Math.min(0.28, fVal * 0.25).toFixed(2) + ')';
              ctx.fillRect(c * CELL_PX + 1, r * CELL_PX + 1, CELL_PX - 1, CELL_PX - 1);
            }
          }
        }

        // 3b. Draw AI Frontier Path Planner Vector Arrow (shows where the map brain wants to go next)
        var goalAngle = -Math.PI / 2 + (latestGoalSteer * 0.52);
        var gxEnd = cx + Math.cos(goalAngle) * 58;
        var gyEnd = cy + Math.sin(goalAngle) * 58;
        ctx.strokeStyle = '#22c55e';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(cx, cy - 14);
        ctx.lineTo(gxEnd, gyEnd);
        ctx.stroke();
        ctx.fillStyle = '#22c55e';
        ctx.beginPath();
        ctx.arc(gxEnd, gyEnd, 4, 0, Math.PI * 2);
        ctx.fill();

        // 4. Draw Top-Down RC Rover Graphic in Center with Animated Steering Wheels
        ctx.save();
        ctx.translate(cx, cy);

        // Rear Tires
        ctx.fillStyle = '#94a3b8';
        ctx.fillRect(-11, 6, 4, 9);
        ctx.fillRect(7, 6, 4, 9);

        // Front Steerable Tires
        var wheelAngle = latestSteerDir * 0.42;
        ctx.save();
        ctx.translate(-9, -9);
        ctx.rotate(wheelAngle);
        ctx.fillRect(-2, -4, 4, 8);
        ctx.restore();

        ctx.save();
        ctx.translate(9, -9);
        ctx.rotate(wheelAngle);
        ctx.fillRect(-2, -4, 4, 8);
        ctx.restore();

        // Car Body Shell
        ctx.fillStyle = '#0284c7';
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.roundRect(-8, -15, 16, 30, 5);
        ctx.fill();
        ctx.stroke();

        // Windshield
        ctx.fillStyle = '#0f172a';
        ctx.fillRect(-6, -8, 12, 7);
        ctx.restore();

        // 5. Draw Target Lock Reticle if active
        if (targetLock) {
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2;
          ctx.strokeRect(targetLock.x - 24, targetLock.y - 24, 48, 48);
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
            if (typeof data.drive === 'number') latestDriveDir = data.drive;
            if (typeof data.steer === 'number') latestSteerDir = data.steer;
            if (typeof data.goal === 'number') latestGoalSteer = data.goal;
            if (typeof data.hdg === 'number') latestHeadingDeg = data.hdg;
            if (typeof data.secL === 'number') secLVal.textContent = data.secL.toFixed(0) + ' cm';
            if (typeof data.secC === 'number') secCVal.textContent = data.secC.toFixed(0) + ' cm';
            if (typeof data.secR === 'number') secRVal.textContent = data.secR.toFixed(0) + ' cm';

            stepHeatMapPhysics(latestDriveDir, latestSteerDir, latestFrontCm, latestRearCm);

            if (data.state) {
              latestState = data.state;
              stateVal.textContent = data.state;
            }
            if (typeof data.staIp === 'string') {
              if (data.staIp.length > 0 && data.staIp !== '0.0.0.0') {
                wifiStaBadge.textContent = data.staIp;
                wifiStaBadge.classList.add('online');
                wifiStatusMsg.textContent = 'Connected to ' + (data.staSsid || 'Wi-Fi') + ' at ' + data.staIp + ' (OTA ready: rc-rover.local)';
              } else if (data.staSsid && data.staSsid.length > 0) {
                wifiStaBadge.textContent = 'CONNECTING...';
                wifiStaBadge.classList.remove('online');
                if ( document.activeElement !== wifiSsidInput && !wifiSsidInput.value ) {
                  wifiSsidInput.value = data.staSsid;
                }
              } else {
                wifiStaBadge.textContent = 'AP ONLY';
                wifiStaBadge.classList.remove('online');
              }
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

constexpr size_t MAX_CAM_JPEG_BYTES = 32768;
constexpr int ONCHIP_VIS_W = 40;
constexpr int ONCHIP_VIS_H = 30;
uint8_t* g_camJpegBuf = nullptr;
uint8_t* g_camServeBuf = nullptr;
uint8_t* g_camStageBuf = nullptr;
uint8_t* g_camVisRgbBuf = nullptr;
size_t g_camJpegLen = 0;
size_t g_camServeLen = 0;
uint32_t g_camFrameSeq = 0;
volatile uint32_t g_lastCamClientMs = 0;
volatile bool g_camReachable = false;
volatile bool g_camPort81Open = false;
volatile float g_onChipVisLeftNorm = 0.7f;
volatile float g_onChipVisCenterNorm = 0.7f;
volatile float g_onChipVisRightNorm = 0.7f;
volatile uint32_t g_lastOnChipVisMs = 0;
SemaphoreHandle_t g_camMutex = nullptr;

// Headless On-Chip Vision-Sonar Floor Horizon Analyzer (runs on Core 0 inside camRelayTask)
// Decodes the 320x240 JPEG at 1/8th scale (40x30 RGB565) and computes 16 optical floor-horizon columns
// so the rover steers toward open visual corridors even when no browser dashboard is open!
void unpackRgb565(const uint8_t* buf, int pixelIdx, int& r, int& g, int& b) {
    // esp32-camera jpg2rgb565 outputs big-endian or little-endian 16-bit RGB565; unpack channels scaled to 0..255
    const uint16_t px = static_cast<uint16_t>(buf[pixelIdx * 2]) |
                        (static_cast<uint16_t>(buf[pixelIdx * 2 + 1]) << 8);
    r = ((px >> 11) & 0x1F) << 3;
    g = ((px >> 5)  & 0x3F) << 2;
    b = (px & 0x1F) << 3;
}

void analyzeFrameOnChip(const uint8_t* jpegBuf, size_t jpegLen) {
    if (jpegBuf == nullptr || jpegLen <= 128 || g_camVisRgbBuf == nullptr) {
        return;
    }
    if (!jpg2rgb565(jpegBuf, jpegLen, g_camVisRgbBuf, JPG_SCALE_8X)) {
        return;
    }

    // Camera is mounted inverted (180 deg rotation):
    //   - Floor directly ahead of the front bumper is at raw top rows (ry: 2..5)
    //   - Scanning upward toward the horizon moves from ry = 4 -> 25
    //   - Rover LEFT (col 0..5) maps to raw right x (39..25); Rover RIGHT (col 10..15) maps to raw left x (14..0)
    int refR = 0, refG = 0, refB = 0, count = 0;
    for (int ry = 2; ry <= 5; ++ry) {
        for (int rx = 15; rx <= 24; ++rx) {
            int pr = 0, pg = 0, pb = 0;
            unpackRgb565(g_camVisRgbBuf, ry * ONCHIP_VIS_W + rx, pr, pg, pb);
            refR += pr;
            refG += pg;
            refB += pb;
            ++count;
        }
    }
    if (count > 0) {
        refR /= count;
        refG /= count;
        refB /= count;
    }

    constexpr int COLS = 16;
    float colNorm[COLS];
    for (int c = 0; c < COLS; ++c) {
        int rawX = ONCHIP_VIS_W - 1 - ((c * ONCHIP_VIS_W + (COLS / 2)) / COLS);
        if (rawX < 0) rawX = 0;
        if (rawX >= ONCHIP_VIS_W) rawX = ONCHIP_VIS_W - 1;

        int hitSteps = 22;
        for (int step = 0; step < 22; ++step) {
            const int rawY = 4 + step;
            int pr = 0, pg = 0, pb = 0;
            unpackRgb565(g_camVisRgbBuf, rawY * ONCHIP_VIS_W + rawX, pr, pg, pb);
            const int dr = abs(pr - refR);
            const int dg = abs(pg - refG);
            const int db = abs(pb - refB);
            if (dr + dg + db > 75) {
                hitSteps = step;
                break;
            }
        }
        colNorm[c] = fmaxf(0.10f, static_cast<float>(hitSteps) / 22.0f);
    }

    float leftSum = 0.0f, centerSum = 0.0f, rightSum = 0.0f;
    for (int i = 0; i < 6; ++i) leftSum += colNorm[i];
    for (int i = 6; i <= 9; ++i) centerSum += colNorm[i];
    for (int i = 10; i < 16; ++i) rightSum += colNorm[i];

    g_onChipVisLeftNorm   = leftSum / 6.0f;
    g_onChipVisCenterNorm = centerSum / 4.0f;
    g_onChipVisRightNorm  = rightSum / 6.0f;
    g_lastOnChipVisMs     = millis();
}

void camRelayTask(void* param) {
    (void)param;
    uint32_t lastProbeMs = 0;

    for (;;) {
        const uint32_t nowMs = millis();
        const uint8_t stations = WiFi.softAPgetStationNum();
        const bool clientStreaming = (g_lastCamClientMs > 0) && (nowMs - g_lastCamClientMs < 3500);
        // Poll at ~4 FPS continuously whenever ESP32-CAM is connected so autonomous navigation
        // always has live on-chip optical floor-horizon steering even with no browser open!
        const bool shouldPoll = (stations > 0) && (clientStreaming || (nowMs - lastProbeMs >= 250));

        if (!shouldPoll) {
            if (stations == 0) {
                g_camReachable = false;
                g_camPort81Open = false;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        lastProbeMs = nowMs;

        // Fetch single JPEG frame from ESP32-CAM http://192.168.4.2/capture
        WiFiClient client;
        client.setTimeout(600);
        if (client.connect(IPAddress(192, 168, 4, 2), 80)) {
            client.print("GET /capture HTTP/1.0\r\nHost: 192.168.4.2\r\nConnection: close\r\n\r\n");

            uint32_t waitStart = millis();
            while (client.connected() && !client.available() && (millis() - waitStart < 600)) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }

            // Skip HTTP response headers until \r\n\r\n
            bool headersDone = false;
            int matchState = 0;
            while (client.connected() || client.available()) {
                if (!client.available()) {
                    if (millis() - waitStart > 800) break;
                    vTaskDelay(pdMS_TO_TICKS(2));
                    continue;
                }
                int c = client.read();
                if (c < 0) break;
                if ((matchState == 0 || matchState == 2) && c == '\r') {
                    matchState++;
                } else if ((matchState == 1 || matchState == 3) && c == '\n') {
                    matchState++;
                    if (matchState == 4) {
                        headersDone = true;
                        break;
                    }
                } else {
                    matchState = (c == '\r') ? 1 : 0;
                }
            }

            if (headersDone && g_camStageBuf != nullptr) {
                size_t totalRead = 0;
                uint32_t readStart = millis();
                while ((client.connected() || client.available()) && totalRead < MAX_CAM_JPEG_BYTES) {
                    int avail = client.available();
                    if (avail > 0) {
                        size_t toRead = static_cast<size_t>(avail);
                        if (totalRead + toRead > MAX_CAM_JPEG_BYTES) {
                            toRead = MAX_CAM_JPEG_BYTES - totalRead;
                        }
                        int got = client.read(g_camStageBuf + totalRead, toRead);
                        if (got > 0) {
                            totalRead += static_cast<size_t>(got);
                            if (totalRead >= 4 &&
                                g_camStageBuf[totalRead - 2] == 0xFF &&
                                g_camStageBuf[totalRead - 1] == 0xD9) {
                                break;
                            }
                        }
                    } else {
                        if (millis() - readStart > 600) break;
                        vTaskDelay(pdMS_TO_TICKS(2));
                    }
                }

                if (totalRead > 128 &&
                    g_camStageBuf[0] == 0xFF && g_camStageBuf[1] == 0xD8 &&
                    g_camStageBuf[totalRead - 2] == 0xFF && g_camStageBuf[totalRead - 1] == 0xD9) {
                    if (g_camMutex != nullptr && xSemaphoreTake(g_camMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        memcpy(g_camJpegBuf, g_camStageBuf, totalRead);
                        g_camJpegLen = totalRead;
                        g_camFrameSeq++;
                        xSemaphoreGive(g_camMutex);
                    }
                    analyzeFrameOnChip(g_camStageBuf, totalRead);
                    g_camReachable = true;
                    g_camPort81Open = true;
                }
            }
            client.stop();
        } else {
            // Fallback check on port 81 in case camera is running older stream-only firmware
            WiFiClient sClient;
            sClient.setTimeout(350);
            if (sClient.connect(IPAddress(192, 168, 4, 2), 81)) {
                g_camPort81Open = true;
                g_camReachable = true;
                sClient.stop();
            } else {
                g_camPort81Open = false;
                g_camReachable = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(clientStreaming ? 75 : 220));
    }
}

} // namespace

WebInterface::WebInterface()
    : _server(80), _ws("/ws") {}

void WebInterface::begin() {
    g_camMutex = xSemaphoreCreateMutex();
    g_camJpegBuf = static_cast<uint8_t*>(malloc(MAX_CAM_JPEG_BYTES));
    g_camServeBuf = static_cast<uint8_t*>(malloc(MAX_CAM_JPEG_BYTES));
    g_camStageBuf = static_cast<uint8_t*>(malloc(MAX_CAM_JPEG_BYTES));
    g_camVisRgbBuf = static_cast<uint8_t*>(malloc(ONCHIP_VIS_W * ONCHIP_VIS_H * 3));

    // Enable Simultaneous SoftAP (192.168.4.1 for ESP32-CAM & direct phone connect) + Home Wi-Fi Station (for OTA)
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASS);

    connectSavedWifi();

    xTaskCreatePinnedToCore(camRelayTask, "camRelay", 8192, nullptr, 1, nullptr, 0);

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
        request->send(200, "text/html", INDEX_HTML);
    });

    _server.on("/cam-status", HTTP_GET, [](AsyncWebServerRequest* request) {
        g_lastCamClientMs = millis();
        const uint8_t stations = WiFi.softAPgetStationNum();
        char buf[260];
        snprintf(buf,
                 sizeof(buf),
                 "{\"apStations\":%u,\"camReachable\":%s,\"port81\":%s,\"jpegBytes\":%u,\"frameSeq\":%lu,\"visL\":%.2f,\"visC\":%.2f,\"visR\":%.2f,\"camUrl\":\"http://192.168.4.2:81/stream\"}",
                 static_cast<unsigned>(stations),
                 g_camReachable ? "true" : "false",
                 g_camPort81Open ? "true" : "false",
                 static_cast<unsigned>(g_camJpegLen),
                 static_cast<unsigned long>(g_camFrameSeq),
                 g_onChipVisLeftNorm,
                 g_onChipVisCenterNorm,
                 g_onChipVisRightNorm);
        request->send(200, "application/json", buf);
    });

    _server.on("/cam.jpg", HTTP_GET, [](AsyncWebServerRequest* request) {
        g_lastCamClientMs = millis();
        if (g_camJpegBuf != nullptr && g_camServeBuf != nullptr && g_camJpegLen > 0 && g_camMutex != nullptr) {
            if (xSemaphoreTake(g_camMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                memcpy(g_camServeBuf, g_camJpegBuf, g_camJpegLen);
                g_camServeLen = g_camJpegLen;
                const size_t len = g_camServeLen;
                xSemaphoreGive(g_camMutex);

                AsyncWebServerResponse* response = request->beginResponse(
                    "image/jpeg",
                    len,
                    [](uint8_t* buffer, size_t maxLen, size_t index) -> size_t {
                        size_t avail = (index < g_camServeLen) ? (g_camServeLen - index) : 0;
                        if (avail > maxLen) avail = maxLen;
                        if (avail > 0) {
                            memcpy(buffer, g_camServeBuf + index, avail);
                        }
                        return avail;
                    });
                response->addHeader("Access-Control-Allow-Origin", "*");
                response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
                request->send(response);
                return;
            }
        }
        request->send(503, "text/plain", "Camera frame not ready");
    });

    _server.begin();
    _lastHeartbeatMs = millis();
}

void WebInterface::connectSavedWifi() {
    Preferences prefs;
    if (prefs.begin("rover-wifi", true)) {
        String savedSsid = prefs.getString("ssid", "");
        String savedPass = prefs.getString("pass", "");
        prefs.end();

        if (savedSsid.length() > 0) {
            strncpy(_staSsid, savedSsid.c_str(), sizeof(_staSsid) - 1);
            _staSsid[sizeof(_staSsid) - 1] = '\0';
            strncpy(_staPass, savedPass.c_str(), sizeof(_staPass) - 1);
            _staPass[sizeof(_staPass) - 1] = '\0';
            WiFi.setHostname("rc-rover");
            WiFi.begin(_staSsid, _staPass);
            _wifiConnectStartMs = millis();
            Serial.printf("[Wi-Fi] Connecting to saved SSID: %s\n", _staSsid);
        }
    }
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
    if (len == 0 || len >= 120) {
        return;
    }

    char buf[120];
    memcpy(buf, data, len);
    buf[len] = '\0';

    _lastHeartbeatMs = millis();

    if (strcmp(buf, "HB") == 0) {
        return;
    }

    // Any explicit command from the web UI cancels the 30-second boot standby timer
    _userInteracted = true;

    if (strncmp(buf, "WIFI:", 5) == 0) {
        char* tabPos = strchr(buf + 5, '\t');
        if (tabPos != nullptr) {
            *tabPos = '\0';
            const char* ssid = buf + 5;
            const char* pass = tabPos + 1;
            if (strlen(ssid) > 0 && strlen(ssid) < sizeof(_staSsid)) {
                strncpy(_staSsid, ssid, sizeof(_staSsid) - 1);
                _staSsid[sizeof(_staSsid) - 1] = '\0';
                strncpy(_staPass, pass, sizeof(_staPass) - 1);
                _staPass[sizeof(_staPass) - 1] = '\0';
                _pendingWifiConnect = true;
            }
        }
        return;
    }

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

    if (strncmp(buf, "VIS:", 4) == 0) {
        char* firstColon = buf + 3;
        char* secondColon = strchr(firstColon + 1, ':');
        if (secondColon != nullptr) {
            *secondColon = '\0';
            float leftCm = static_cast<float>(atof(firstColon + 1));
            float rightCm = static_cast<float>(atof(secondColon + 1));
            if (leftCm >= 5.0f && leftCm <= 250.0f && rightCm >= 5.0f && rightCm <= 250.0f) {
                _visionLeftCm = leftCm;
                _visionRightCm = rightCm;
                _lastVisionUpdateMs = millis();
            }
        }
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

bool WebInterface::hasVisionSectorFusion() const {
    const uint32_t nowMs = millis();
    const bool hasBrowserVis = (_lastVisionUpdateMs > 0) && (nowMs - _lastVisionUpdateMs <= 1000);
    const bool hasOnChipVis = (g_lastOnChipVisMs > 0) && (nowMs - g_lastOnChipVisMs <= 1500);
    return hasBrowserVis || hasOnChipVis;
}

bool WebInterface::isManualWatchdogExpired() const {
    return (millis() - _lastHeartbeatMs) > WATCHDOG_TIMEOUT_MS;
}

void WebInterface::clearManualCommand() {
    _manualCmd.drive = 0;
    _manualCmd.steer = 0;
}

void WebInterface::broadcastTelemetry(const TelemetrySnapshot& t) {
    if (_ws.count() == 0) {
        return;
    }

    IPAddress staIp = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : IPAddress(0, 0, 0, 0);
    char ipBuf[20];
    if (WiFi.status() == WL_CONNECTED) {
        snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u", staIp[0], staIp[1], staIp[2], staIp[3]);
    } else {
        ipBuf[0] = '\0';
    }

    char json[340];
    snprintf(json,
             sizeof(json),
             "{\"front\":%.1f,\"rear\":%.1f,\"drive\":%d,\"steer\":%d,\"secL\":%.0f,\"secC\":%.0f,\"secR\":%.0f,\"hdg\":%.0f,\"goal\":%d,\"state\":\"%s\",\"wd\":%s,\"mode\":%u,\"staIp\":\"%s\",\"staSsid\":\"%s\"}",
             t.frontCm,
             t.rearCm,
             static_cast<int>(t.driveDir),
             static_cast<int>(t.steerDir),
             t.sectorLeftCm,
             t.sectorCenterCm,
             t.sectorRightCm,
             t.headingDeg,
             static_cast<int>(t.goalSteer),
             t.stateLabel,
             t.watchdogTripped ? "true" : "false",
             static_cast<unsigned>(_mode),
             ipBuf,
             _staSsid);
    _ws.textAll(json);
}

void WebInterface::update(const TelemetrySnapshot& telemetry) {
    // Keep headless on-chip vision clearances calibrated against live Front Sonar distance
    // whenever no browser is actively streaming VIS:<left>:<right> packets
    const uint32_t nowMs = millis();
    if ((_lastVisionUpdateMs == 0 || (nowMs - _lastVisionUpdateMs > 1000)) &&
        g_lastOnChipVisMs > 0 && (nowMs - g_lastOnChipVisMs <= 1500)) {
        const float clampedSonar = fminf(fmaxf(telemetry.frontCm, 12.0f), 140.0f);
        const float scaleCm = clampedSonar / fmaxf(g_onChipVisCenterNorm, 0.15f);
        _visionLeftCm  = fminf(fmaxf(g_onChipVisLeftNorm * scaleCm, 10.0f), 150.0f);
        _visionRightCm = fminf(fmaxf(g_onChipVisRightNorm * scaleCm, 10.0f), 150.0f);
    }

    _ws.cleanupClients();

    if (_pendingWifiConnect) {
        _pendingWifiConnect = false;
        Preferences prefs;
        if (prefs.begin("rover-wifi", false)) {
            prefs.putString("ssid", _staSsid);
            prefs.putString("pass", _staPass);
            prefs.end();
        }
        WiFi.setHostname("rc-rover");
        WiFi.disconnect(false);
        WiFi.begin(_staSsid, _staPass);
        _wifiConnectStartMs = millis();
        Serial.printf("[Wi-Fi] Saved & connecting to SSID: %s\n", _staSsid);
    }

    if (nowMs - _lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        _lastTelemetryMs = nowMs;
        broadcastTelemetry(telemetry);
    }
}
