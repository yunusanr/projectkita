// Memilih elemen video untuk kedua webcam
const video1 = document.getElementById('webcam1');
const video2 = document.getElementById('webcam2');
const videoElement = video1; // Kamera yang digunakan untuk snapshot
const canvas = document.getElementById("canvas");
const timerElement = document.getElementById("timer");
const hourInput = document.getElementById("hourInput");
const minuteInput = document.getElementById("minuteInput");
const secondInput = document.getElementById("secondInput");
const startButton = document.getElementById("startButton");
const context = canvas.getContext("2d");

let timerInterval;
let secondsLeft;

// Fungsi untuk memulai streaming dari kamera pertama yang ditemukan
async function startWebcam(videoElement) {
    try {
        const stream = await navigator.mediaDevices.getUserMedia({
            video: true // Mengakses kamera pertama yang tersedia
        });
        videoElement.srcObject = stream;
    } catch (error) {
        console.error("Tidak dapat mengakses webcam:", error);
    }
}

// Fungsi untuk mendeteksi dan memulai kedua kamera jika ada
async function initDualWebcams() {
    try {
        // Mendapatkan daftar perangkat media
        const devices = await navigator.mediaDevices.enumerateDevices();
        const videoDevices = devices.filter(device => device.kind === 'videoinput');

        // Jika ada dua kamera, mulai streaming di masing-masing elemen video
        if (videoDevices.length >= 2) {
            // Start webcam untuk kamera pertama
            const stream1 = await navigator.mediaDevices.getUserMedia({
                video: { deviceId: videoDevices[0].deviceId }
            });
            video1.srcObject = stream1;

            // Start webcam untuk kamera kedua
            const stream2 = await navigator.mediaDevices.getUserMedia({
                video: { deviceId: videoDevices[1].deviceId }
            });
            video2.srcObject = stream2;
        } else if (videoDevices.length === 1) {
            startWebcam(video1); // Hanya satu kamera, tampilkan di webcam1
            console.log("Hanya satu kamera yang tersedia.");
        } else {
            console.error("Tidak ditemukan kamera.");
        }
    } catch (error) {
        console.error("Gagal mendeteksi perangkat media:", error);
    }
}


// Memulai proses deteksi kamera saat halaman dimuat
window.onload = initDualWebcams;

// Function to update the timer display
function updateTimer() {
    const hours = String(Math.floor(secondsLeft / 3600)).padStart(2, "0");
    const minutes = String(Math.floor((secondsLeft % 3600) / 60)).padStart(2, "0");
    const seconds = String(secondsLeft % 60).padStart(2, "0");
    timerElement.textContent = `${hours}::${minutes}::${seconds}`;
}

// Function to take a snapshot
// Function to take snapshots from both cameras
function takeSnapshot() {
    // Set canvas size to match video for webcam 1
    canvas.width = video1.videoWidth;
    canvas.height = video1.videoHeight;
    
    // Draw the current frame from video1 on the canvas
    context.drawImage(video1, 0, 0, canvas.width, canvas.height);

    // Convert the canvas to a data URL (image) for webcam 1
    const dataUrl1 = canvas.toDataURL("image/png");

    // Set canvas size to match video for webcam 2
    canvas.width = video2.videoWidth;
    canvas.height = video2.videoHeight;
    
    // Draw the current frame from video2 on the canvas
    context.drawImage(video2, 0, 0, canvas.width, canvas.height);

    // Convert the canvas to a data URL (image) for webcam 2
    const dataUrl2 = canvas.toDataURL("image/png");

    // Get current date and time for file names
    const now = new Date();
    const day = String(now.getDate()).padStart(2, "0");
    const month = String(now.getMonth() + 1).padStart(2, "0");
    const year = now.getFullYear();
    const hours = String(now.getHours()).padStart(2, "0");
    const minutes = String(now.getMinutes()).padStart(2, "0");
    const seconds = String(now.getSeconds()).padStart(2, "0");

    // Create the file names for both snapshots
    const fileName1 = `kamera1-${day}-${month}-${year}_${hours}-${minutes}-${seconds}.png`;
    const fileName2 = `kamera2-${day}-${month}-${year}_${hours}-${minutes}-${seconds}.png`;

    // Create links to trigger the download for both images
    const link1 = document.createElement("a");
    link1.href = dataUrl1;
    link1.download = fileName1;
    link1.click();

    const link2 = document.createElement("a");
    link2.href = dataUrl2;
    link2.download = fileName2;
    link2.click();
}


// Timer function to count down
function startTimer() {
    // Get the time from the input boxes (in seconds)
    const hours = parseInt(hourInput.value) || 0;
    const minutes = parseInt(minuteInput.value) || 0;
    const seconds = parseInt(secondInput.value) || 0;

    // Convert the timer value to total seconds
    secondsLeft = hours * 3600 + minutes * 60 + seconds;

    // Start counting down
    timerInterval = setInterval(() => {
        if (secondsLeft > 0) {
            secondsLeft--;
            updateTimer();
        } else {
            // Take snapshot when timer reaches zero
            takeSnapshot();
            clearInterval(timerInterval); // Stop the timer after snapshot is taken
        }
    }, 1000); // Update every second
}

// Start the timer when the button is clicked
startButton.addEventListener("click", () => {
    clearInterval(timerInterval); // Clear any existing timer
    startTimer();
});
