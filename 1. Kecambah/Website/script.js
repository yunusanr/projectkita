const canvas = document.querySelector('.graphic');
const ctx = canvas.getContext('2d');

new Swiper('.card-wrapper', {
    loop: true,          // Mengaktifkan looping slide
    spaceBetween: 85,    // Memberi jarak antar-slide
    pagination: {
        el: '.swiper-pagination',   // Element untuk pagination
        clickable: true,            // Membuat bullet pagination bisa diklik
        dynamicBullets: true        // Bullet berubah ukuran sesuai slide aktif
    },
    navigation: {
        nextEl: '.swiper-button-next',  // Tombol next
        prevEl: '.swiper-button-prev',  // Tombol prev
    },
    breakpoints: {
        0: {
            slidesPerView: 1,       // 1 slide per layar untuk layar kecil (mobile)
        },
        768: {
            slidesPerView: 2,       // 2 slide per layar untuk tablet
        },
        1024: {
            slidesPerView: 3,       // 3 slide per layar untuk desktop
        },
    }
});

// Inisialisasi Chart.js
const myChart = new Chart(ctx, {
    type: 'bar',
    data: {
        labels: ['A', 'B', 'C', 'D'], // Label sumbu X
        datasets: [{
            label: 'Data', 
            data: [5, 10, 15, 20], // Nilai awal
            backgroundColor: [
                'rgba(255, 99, 132, 0.6)',
                'rgba(54, 162, 235, 0.6)',
                'rgba(255, 206, 86, 0.6)',
                'rgba(75, 192, 192, 0.6)'
            ],
            borderColor: [
                'rgba(255, 99, 132, 1)',
                'rgba(54, 162, 235, 1)',
                'rgba(255, 206, 86, 1)',
                'rgba(75, 192, 192, 1)'
            ],
            borderWidth: 1
        }]
    },
    options: {
        responsive: false,
        maintainAspectRatio: false,
        scales: {
            y: {
                beginAtZero: true,
                ticks: {
                    color: 'rgba(0, 123, 255, 1)' // Warna label sumbu Y
                }
            },
            x: {
                ticks: {
                    color: 'rgba(255, 159, 64, 1)' // Warna label sumbu X
                }
            }
        },
        plugins: {
            legend: {
                labels: {
                    color: 'rgba(153, 102, 255, 1)', // Warna label legend
                    font: {
                        size: 16 // Ukuran font legend
                    }
                }
            },
            tooltip: {
                titleColor: 'rgba(0, 200, 83, 1)', // Warna judul tooltip
                bodyColor: 'rgba(233, 30, 99, 1)', // Warna teks tooltip
                backgroundColor: 'rgba(0, 0, 0, 0.7)', // Warna background tooltip
                borderColor: 'rgba(255, 255, 255, 0.8)', // Warna border tooltip
                borderWidth: 1
            }
        }
    }
});

// Fungsi untuk mengupdate data setiap 5 detik
function updateChart() {
    const newData = Array.from({ length: 4 }, () => Math.floor(Math.random() * 25)); // Nilai acak baru

    myChart.data.datasets[0].data = newData; // Update data
    myChart.update(); // Render ulang chart

    console.log(`Data baru: ${newData}`); // Debugging
}

// Jalankan update setiap 5 detik
setInterval(updateChart, 5000);

let progress = 0; // Mulai dari 0%

function updateProgress() {
    const progressBar = document.getElementById("progressBar");
    const circumference = 502; // Keliling lingkaran (2 * Math.PI * r)

    // Hitung dashoffset berdasarkan progress (0-100%)
    const offset = circumference - (progress / 100) * circumference;
    progressBar.style.strokeDashoffset = offset;

    // Update progress setiap kali, reset jika sudah 100%
    progress = (progress + 10) % 110; // Naik 10% tiap 5 detik, reset ke 0 jika mencapai 100%
}

// Panggil fungsi updateProgress setiap 5 detik
setInterval(updateProgress, 5000);

window.addEventListener('load', () => {
    document.getElementById('header').classList.add('show');
});

function scrollToTop() {
    window.scrollTo({
        top: 0,  // Scroll ke posisi paling atas
        behavior: 'smooth'  // Scroll dengan animasi halus
    });
}

window.addEventListener('resize', function () {
    const header = document.getElementById('header');
    if (window.innerWidth <= 1500) {
        header.style.backgroundAttachment = 'scroll';
    } else {
        header.style.backgroundAttachment = 'fixed';
    }
});

