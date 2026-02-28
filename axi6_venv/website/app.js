document.addEventListener('DOMContentLoaded', () => {
    const playhead = document.getElementById('playhead');
    const curveArea = document.getElementById('curve-area');
    const timerDisplay = document.getElementById('timeline-timer');
    const dragIndicator = document.getElementById('drag-indicator');
    const timelineContent = document.getElementById('timeline-content');
    
    let isDragging = false;
    
    // Configuration: Total timeline duration in seconds and framerate
    const durationSeconds = 60; // 1 minute max for now
    const fps = 30;
    const pixelsPerSecond = 100; // 100 pixels per second
    
    if (timelineContent) {
        timelineContent.style.width = `${durationSeconds * pixelsPerSecond}px`;
    }
    
    // Function to calculate and update playhead position and timer text
    function updatePlayheadPosition(clientX) {
        const rect = timelineContent.getBoundingClientRect();
        let x = clientX - rect.left;
        
        // Constrain playhead within the curve area bounds
        if (x < 0) x = 0;
        if (x > rect.width) x = rect.width;
        
        // Move the playhead
        playhead.style.left = `${x}px`;
        
        // Calculate the time based on progress
        const progress = x / rect.width;
        const totalTime = durationSeconds * progress;
        
        // Format time (HH:MM:SS:FF)
        const hours = Math.floor(totalTime / 3600);
        const minutes = Math.floor((totalTime % 3600) / 60);
        const seconds = Math.floor(totalTime % 60);
        const frames = Math.floor((totalTime % 1) * fps);
        
        const formattedTime = 
            String(hours).padStart(2, '0') + ':' +
            String(minutes).padStart(2, '0') + ':' +
            String(seconds).padStart(2, '0') + ':' +
            String(frames).padStart(2, '0');
            
        // Update the display text
        timerDisplay.textContent = formattedTime;
    }
    
    // Start dragging when clicking on the playhead
    playhead.addEventListener('mousedown', (e) => {
        isDragging = true;
        // Prevent highlighting text while dragging
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });
    
    // Auto-scrolling variables
    let scrollAnimationFrame = null;
    let scrollVelocity = 0;
    let lastClientX = 0; // Keep track of JS mouse position
    
    // Update playhead on mouse move across the whole document
    document.addEventListener('mousemove', (e) => {
        if (!isDragging) return;
        // Auto-scrolling logic based on viewport proximity
        const rect = curveArea.getBoundingClientRect();
        
        // Prevent the playhead from visually sliding out of the box while dragging
        let constrainedClientX = e.clientX;
        if (constrainedClientX < rect.left) constrainedClientX = rect.left;
        if (constrainedClientX > rect.right) constrainedClientX = rect.right;
        
        lastClientX = constrainedClientX;
        
        // Calculate the playhead position internally using the constrained X
        updatePlayheadPosition(constrainedClientX);
        
        // Configuration for the auto-scroll speed and trigger zone
        const edgeZone = 80; // Start scrolling 80px before the edge
        const maxScrollSpeed = 25; // Maximum scroll speed
        
        // Distance from edges (negative means mouse is PAST the edge)
        const distFromRight = rect.right - e.clientX;
        const distFromLeft = e.clientX - rect.left;
        
        if (distFromRight < edgeZone) {
            // Mouse is near or past the right edge
            let intensity = 1 - Math.max(0, distFromRight) / edgeZone;
            // Cap intensity at 1 (max speed) if we are way past the edge
            if (intensity > 1) intensity = 1; 
            scrollVelocity = intensity * maxScrollSpeed;
            startAutoScroll();
        } else if (distFromLeft < edgeZone) {
            // Mouse is near or past the left edge
            let intensity = 1 - Math.max(0, distFromLeft) / edgeZone; 
            if (intensity > 1) intensity = 1;
            scrollVelocity = -intensity * maxScrollSpeed;
            startAutoScroll();
        } else {
            // Mouse is strictly inside the safe zone
            stopAutoScroll();
        }
    });
    
    // Continuously scroll the curve area horizontally
    function autoScrollLoop() {
        if (!isDragging || scrollVelocity === 0) {
            stopAutoScroll();
            return;
        }
        
        curveArea.scrollLeft += scrollVelocity;
        
        // Since the window scrolled out from under us, we need to recalculate the playhead position based on that new scroll offset
        updatePlayheadPosition(lastClientX);
        
        scrollAnimationFrame = requestAnimationFrame(autoScrollLoop);
    }
    
    function startAutoScroll() {
        if (!scrollAnimationFrame) {
            autoScrollLoop();
        }
    }
    
    function stopAutoScroll() {
        if (scrollAnimationFrame) {
            cancelAnimationFrame(scrollAnimationFrame);
            scrollAnimationFrame = null;
        }
        scrollVelocity = 0;
    }
    
    // Stop dragging when mouse goes up anywhere
    document.addEventListener('mouseup', () => {
        if (isDragging) {
            isDragging = false;
            stopAutoScroll();
            document.body.style.userSelect = '';
        }
    });

    // Clicking anywhere on the background jumps the playhead to that spot
    timelineContent.addEventListener('mousedown', (e) => {
        if (e.target !== playhead && !playhead.contains(e.target)) {
            updatePlayheadPosition(e.clientX);
            isDragging = true;
            document.body.style.userSelect = 'none';
        }
    });

    // Helper to format time for ruler labels
    function formatRulerTime(totalTime) {
        const hours = Math.floor(totalTime / 3600);
        const minutes = Math.floor((totalTime % 3600) / 60);
        const seconds = Math.floor(totalTime % 60);
        const frames = Math.floor((totalTime % 1) * fps);
        return String(hours).padStart(2, '0') + ':' +
               String(minutes).padStart(2, '0') + ':' +
               String(seconds).padStart(2, '0') + ':' +
               String(frames).padStart(2, '0');
    }

    // Function to draw the timeline ruler tick marks and text
    function drawRuler() {
        const ruler = document.getElementById('timeline-ruler');
        if (!ruler) return;
        ruler.innerHTML = ''; // Clear existing
        
        const width = timelineContent.getBoundingClientRect().width;
        if (width === 0) return; // Not visible yet
        
        // Configuration for ticks
        const majorTickInterval = 2; // 2 second intervals for labels to reduce clutter
        const minorTickInterval = 0.25; // 0.25 second intervals for more frequent small ticks
        
        // Loop through the total duration and generate ticks
        for (let t = 0; t <= durationSeconds; t += minorTickInterval) {
            const percentage = (t / durationSeconds) * 100;
            // Add a tiny epsilon to handle floating point imprecision for module operator
            const isMajor = Math.abs((t % majorTickInterval)) < 0.001 || Math.abs((t % majorTickInterval) - majorTickInterval) < 0.001;
            
            const tick = document.createElement('div');
            tick.className = `absolute bottom-0 w-[1px] bg-white opacity-${isMajor ? '30' : '10'}`;
            tick.style.height = isMajor ? '12px' : '6px';
            tick.style.left = `${percentage}%`;
            
            ruler.appendChild(tick);
            
            // Draw label only for major ticks, and explicitly hide them for the very start and very end
            if (isMajor && t > 0.01 && t < durationSeconds - 0.01) {
                const label = document.createElement('div');
                label.className = 'absolute top-1 text-[9px] font-mono text-white/40 transform -translate-x-1/2';
                label.style.left = `${percentage}%`;
                label.textContent = formatRulerTime(t);
                
                ruler.appendChild(label);
            }
        }
    }
    
    // Draw ruler on load and on window resize
    drawRuler();
    window.addEventListener('resize', drawRuler);
});
