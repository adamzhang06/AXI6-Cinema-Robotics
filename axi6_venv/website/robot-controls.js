/**
 * robot-controls.js — Minimal interaction wiring for Robot Controls panel.
 *
 * Currently handles:
 *   - Jog speed slider readout
 *   - Mode toggle visual state swap
 *
 * No motor commands or WebSocket logic yet.
 */

document.addEventListener('DOMContentLoaded', () => {

    // ── Jog Speed Slider ──
    const jogSlider = document.getElementById('jog-speed-slider');
    const jogReadout = document.getElementById('jog-speed-readout');

    if (jogSlider && jogReadout) {
        jogSlider.addEventListener('input', () => {
            jogReadout.textContent = `${jogSlider.value}%`;
        });
    }

    // ── Mode Toggle ──
    const modeToggle = document.getElementById('mode-toggle');
    const trackingSettings = document.getElementById('tracking-settings');

    if (modeToggle) {
        const segments = modeToggle.querySelectorAll('.mode-seg');

        segments.forEach(seg => {
            seg.addEventListener('click', () => {
                // Check if already active
                const alreadyActive = seg.classList.contains('mode-seg-active');

                // Swap active state
                segments.forEach(s => s.classList.remove('mode-seg-active'));
                seg.classList.add('mode-seg-active');

                // Toggle dynamic settings visibility
                const mode = seg.getAttribute('data-mode');
                if (trackingSettings) {
                    if (mode === 'tracking') {
                        trackingSettings.style.display = 'flex';
                        
                        // Turn on camera if not already active and mode was just switched to Tracking
                        if (!alreadyActive) {
                            const btnToggleCamera = document.getElementById('btn-toggle-camera');
                            // If it doesn't have text-white, it means camera is off (it has text-white/50)
                            if (btnToggleCamera && !btnToggleCamera.classList.contains('text-white')) {
                                btnToggleCamera.click();
                            }
                        }
                    } else {
                        trackingSettings.style.display = 'none';
                    }
                }
            });
        });
    }
});
