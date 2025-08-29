const ws = new WebSocket("ws://192.168.4.1:80/ws");
let telemetryData = []; // Accumulate all samples here

function clearGraphs() {
  telemetryData = [];
  Plotly.purge("graph_attitude");
  Plotly.purge("graph_pitch_rate");
  Plotly.purge("graph_yaw_rate");
  initPlots();
}

function initPlots() {
  // Attitude
  Plotly.newPlot(
    "graph_attitude",
    [
      { x: [], y: [], mode: "lines", name: "Pitch" },
      { x: [], y: [], mode: "lines", name: "Yaw" },
    ],
    {
      title: "Attitude Estimates",
      xaxis: { title: "Time (ms)" },
      yaxis: { title: "Angle (deg)" },
    }
  );

  // Pitch rate
  Plotly.newPlot(
    "graph_pitch_rate",
    [
      { x: [], y: [], mode: "lines", name: "Pitch Rate SP" },
      { x: [], y: [], mode: "lines", name: "Pitch Rate Actual" },
    ],
    {
      title: "Pitch Rate",
      xaxis: { title: "Time (ms)" },
      yaxis: { title: "Rate (deg/s)" },
    }
  );

  // Yaw rate
  Plotly.newPlot(
    "graph_yaw_rate",
    [
      { x: [], y: [], mode: "lines", name: "Yaw Rate SP" },
      { x: [], y: [], mode: "lines", name: "Yaw Rate Actual" },
    ],
    {
      title: "Yaw Rate",
      xaxis: { title: "Time (ms)" },
      yaxis: { title: "Rate (deg/s)" },
    }
  );
}

initPlots();

function handleTelemetry(data) {
  if (data && Array.isArray(data)) {
    // Append batch to accumulation

    const newPoints = data.map((d) => ({
      timestamp: d[0],
      attitude: { roll: d[1], pitch: d[2], yaw: d[3] },
      imu: {
        ax: d[4],
        ay: d[5],
        az: d[6],
        gx: d[7],
        gy: d[8],
        gz: d[9],
      },
      barometer: { pressure: d[10], temperature: d[11] },
      rate_sp: { roll: d[12], pitch: d[13], yaw: d[14] },
      thrust_sp: { pitch: d[15], roll: d[16], yaw: d[17] },
    }));

    telemetryData = telemetryData.concat(newPoints);

    // Update Plotly graph for attitude estimates
    const times = newPoints.map((d) => d.timestamp);

    const pitches = newPoints.map((d) => d.attitude.pitch);
    const yaws = newPoints.map((d) => d.attitude.yaw);

    const pitch_rate_sp = newPoints.map((d) => d.rate_sp.pitch);
    const yaw_rate_sp = newPoints.map((d) => d.rate_sp.yaw);

    const pitch_rate = newPoints.map((d) => d.imu.gx);
    const yaw_rate = newPoints.map((d) => d.imu.gz);

    Plotly.extendTraces(
      "graph_attitude",
      { x: [times, times], y: [pitches, yaws] },
      [0, 1]
    );

    Plotly.extendTraces(
      "graph_pitch_rate",
      { x: [times, times], y: [pitch_rate_sp, pitch_rate] },
      [0, 1]
    );

    Plotly.extendTraces(
      "graph_yaw_rate",
      { x: [times, times], y: [yaw_rate_sp, yaw_rate] },
      [0, 1]
    );
  }
}

function populateForms(data) {
  for (const [name, cfg] of Object.entries(data)) {
    // round the values to 4 decimal places for better display
    cfg.kp = Math.round(cfg.kp * 10000) / 10000;
    cfg.ki = Math.round(cfg.ki * 10000) / 10000;
    cfg.kd = Math.round(cfg.kd * 10000) / 10000;
    document.getElementById(`${name}_p`).value = cfg.kp;
    document.getElementById(`${name}_i`).value = cfg.ki;
    document.getElementById(`${name}_d`).value = cfg.kd;
  }
}

function handlePidAck(data) {
  for (const [name, cfg] of Object.entries(data)) {
    document.getElementById(`ack_${name}`).innerText = "Updated!";
    // optional: repopulate values to confirm what was saved
    // document.getElementById(`${name}_p`).value = cfg.kp;
    // document.getElementById(`${name}_i`).value = cfg.ki;
    // document.getElementById(`${name}_d`).value = cfg.kd;
  }
}

ws.onopen = () => {
  console.log("WebSocket connected");
  document.getElementById("status").innerText = "Connected to ESP";
};
ws.onmessage = (event) => {
  try {
    const msg = JSON.parse(event.data);

    switch (msg.type) {
      case "telemetry":
        handleTelemetry(msg.data);
        break;
      case "pid_config":
        console.log("Received PID config:", msg.data);
        populateForms(msg.data);
        break;
      case "pid_ack":
        handlePidAck(msg.data);
        break;
      case "status":
        document.getElementById("status").innerText = msg.data;
        break;
      default:
        console.warn("Unknown message type:", msg.type);
    }
  } catch (e) {
    console.error("Error while processing WebSocket message:", e);
  }
};

ws.onclose = () => {
  console.log("WebSocket closed");
  document.getElementById("status").innerText = "Disconnected";
};

function sendPids(name) {
  const msg = {
    [name]: {
      kp: parseFloat(document.getElementById(`${name}_p`).value),
      ki: parseFloat(document.getElementById(`${name}_i`).value),
      kd: parseFloat(document.getElementById(`${name}_d`).value),
    },
  };
  ws.send(JSON.stringify(msg));
  document.getElementById("status").innerText = "PID updates sent";
}
function downloadCsv() {
  if (telemetryData.length === 0) return alert("No data to download");
  // Generate CSV header and rows for all fields
  let csv =
    "timestamp,attitude_roll,attitude_pitch,attitude_yaw,imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,barometer_pressure,barometer_temperature,rate_setpoint_roll,rate_setpoint_pitch,rate_setpoint_yaw,thrust_setpoint_pitch,thrust_setpoint_roll,thrust_setpoint_yaw\n";
  telemetryData.forEach((d) => {
    csv += `${d.timestamp},${d.attitude.roll},${d.attitude.pitch},${d.attitude.yaw},${d.imu.ax},${d.imu.ay},${d.imu.az},${d.imu.gx},${d.imu.gy},${d.imu.gz},${d.barometer.pressure},${d.barometer.temperature},${d.rate_setpoint.roll},${d.rate_setpoint.pitch},${d.rate_setpoint.yaw},${d.thrust_setpoint.pitch},${d.thrust_setpoint.roll},${d.thrust_setpoint.yaw}\n`;
  });
  const blob = new Blob([csv], { type: "text/csv" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "drone_telemetry.csv";
  a.click();
  URL.revokeObjectURL(url);
}
