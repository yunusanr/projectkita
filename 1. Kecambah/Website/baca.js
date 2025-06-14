// Inisialisasi Chart.js untuk suhu
const ctxTemperature = document.getElementById("temperatureChart").getContext("2d");
const temperatureChart = new Chart(ctxTemperature, {
  type: "line",
  data: {
    labels: [], // Waktu akan ditambahkan secara dinamis
    datasets: [
      {
        label: "Suhu (°C)",
        data: [],
        borderColor: "rgba(255, 99, 132, 1)",
        backgroundColor: "rgba(255, 99, 132, 0.2)",
        fill: true,
        tension: 0.1,
      },
    ],
  },
  options: {
    responsive: true,
    scales: {
      x: { title: { display: true, text: "Waktu (s)" } },
      y: {
        title: { display: true, text: "Suhu (°C)" },
        min: 0,
        max: 100,
      },
    },
  },
});

// Inisialisasi Chart.js untuk kelembapan
const ctxHumidity = document.getElementById("humidityChart").getContext("2d");
const humidityChart = new Chart(ctxHumidity, {
  type: "line",
  data: {
    labels: [],
    datasets: [
      {
        label: "Kelembapan (%)",
        data: [],
        borderColor: "rgba(54, 162, 235, 1)",
        backgroundColor: "rgba(54, 162, 235, 0.2)",
        fill: true,
        tension: 0.1,
      },
    ],
  },
  options: {
    responsive: true,
    scales: {
      x: { title: { display: true, text: "Waktu (s)" } },
      y: {
        title: { display: true, text: "Kelembapan (%)" },
        min: 0,
        max: 100,
      },
    },
  },
});

// WebSocket connection
let websocket = new WebSocket("ws://192.168.1.24:8765");

let storedValue = "";
let storedHumidityValue = "";

// Fungsi untuk menambahkan karakter ke input suhu
function addCharacter(char) {
  const inputField = document.getElementById("inputField");
  inputField.value += char;
}

// Fungsi untuk menambahkan karakter ke input kelembapan
function addCharacter2(char) {
  const inputField2 = document.getElementById("inputField2");
  inputField2.value += char;
}

// Fungsi untuk menghapus input suhu
function clearInput() {
  document.getElementById("inputField").value = "";
}

// Fungsi untuk menghapus input kelembapan
function clearInput2() {
  document.getElementById("inputField2").value = "";
}

// Fungsi untuk menyimpan suhu
function saveValue() {
  const inputField = document.getElementById("inputField");
  storedValue = inputField.value;

  // Tampilkan ke target suhu
  document.getElementById("targetSuhu").textContent = storedValue;

  alert("Celcius Tersett: " + storedValue);

  // Mengirimkan nilai ke WebSocket
  const message = { type: "setTemperature", value: storedValue };

  if (websocket.readyState === WebSocket.OPEN) {
    websocket.send(JSON.stringify(message));
    console.log("Nilai suhu terkirim ke WebSocket:", message);
  } else {
    console.log("WebSocket tidak terhubung.");
  }

  inputField.value = ""; // Bersihkan input
}

// Fungsi untuk menyimpan kelembapan
function saveValue2() {
  const inputField2 = document.getElementById("inputField2");
  storedHumidityValue = inputField2.value;

  // Tampilkan ke target kelembapan
  document.getElementById("targetKelembapan").textContent = storedHumidityValue;

  alert("Humidity Tersett: " + storedHumidityValue);

  // Mengirimkan nilai ke WebSocket
  const message = { type: "setHumidity", value: storedHumidityValue };

  if (websocket.readyState === WebSocket.OPEN) {
    websocket.send(JSON.stringify(message));
    console.log("Nilai kelembapan terkirim ke WebSocket:", message);
  } else {
    console.log("WebSocket tidak terhubung.");
  }

  inputField2.value = ""; // Bersihkan input
}

// Fungsi untuk memperbarui grafik
function updateCharts(data) {
  const time = new Date().toLocaleTimeString();

  // Update suhu
  if (temperatureChart.data.labels.length > 20) {
    temperatureChart.data.labels.shift();
    temperatureChart.data.datasets[0].data.shift();
  }
  temperatureChart.data.labels.push(time);
  temperatureChart.data.datasets[0].data.push(data.temperature);
  temperatureChart.update();

  // Update kelembapan
  if (humidityChart.data.labels.length > 20) {
    humidityChart.data.labels.shift();
    humidityChart.data.datasets[0].data.shift();
  }
  humidityChart.data.labels.push(time);
  humidityChart.data.datasets[0].data.push(data.humidity);
  humidityChart.update();
}

// Handle pesan WebSocket
websocket.onmessage = function (event) {
  let data = JSON.parse(event.data);
  document.getElementById("suhu").textContent = data.temperature;
  document.getElementById("kelembapan").textContent = data.humidity;
  document.getElementById("fan1_status").textContent = data.fan1_status ? "ON" : "OFF";
  document.getElementById("fan2_status").textContent = data.fan2_status ? "ON" : "OFF";

  // Update grafik
  updateCharts(data);
};

// Fungsi toggle relay dengan localStorage
function setupToggle(buttonId, relayName) {
  const button = document.getElementById(buttonId);
  const icon = button.querySelector("i");

  // Ambil status dari localStorage
  let isOn = localStorage.getItem(relayName) === "1";

  // Set tampilan awal sesuai status yang tersimpan
  if (isOn) {
    button.classList.add("button-clicked");
    icon.classList.add("icon-clicked");
  } else {
    button.classList.remove("button-clicked");
    icon.classList.remove("icon-clicked");
  }

  // Kirim status awal ke WebSocket jika terhubung
  if (websocket.readyState === WebSocket.OPEN) {
    let initialMessage = {};
    initialMessage[relayName] = isOn ? 1 : 0;
    websocket.send(JSON.stringify(initialMessage));
    console.log("Status relay saat ini terkirim:", initialMessage);
  }

  // Tambahkan event listener untuk toggle relay
  button.addEventListener("click", function () {
    isOn = !isOn;
    button.classList.toggle("button-clicked", isOn);
    icon.classList.toggle("icon-clicked", isOn);

    // Simpan status ke localStorage
    localStorage.setItem(relayName, isOn ? "1" : "0");

    // Kirim status baru ke WebSocket
    let message = {};
    message[relayName] = isOn ? 1 : 0;

    if (websocket.readyState === WebSocket.OPEN) {
      websocket.send(JSON.stringify(message));
      console.log("Status relay terkirim:", message);
    }
  });
}

setupToggle("relay_a_toggle", "relay_a");
setupToggle("relay_b_toggle", "relay_b");
setupToggle("relay_c_toggle", "relay_c");
setupToggle("relay_d_toggle", "relay_d");

// Fungsi untuk menampilkan menu
function openmenu() {
  document.getElementById("submenu").style.right = "0";
}

function closemenu() {
  document.getElementById("submenu").style.right = "-200px";
}
