#!/usr/bin/env python3
"""
Web-based Visualizer for Posha Robotics Assignment
Shows robot movement in a web browser
"""

import math
import time
import webbrowser
import os

class WebVisualizer:
    def __init__(self):
        self.operations = []
        
    def create_web_visualization(self):
        """Create HTML file with animated visualization"""
        html_content = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Posha Robotics Assignment - Live Visualization</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                .container { max-width: 1000px; margin: 0 auto; }
                .workspace { 
                    width: 800px; 
                    height: 600px; 
                    border: 2px solid #333; 
                    position: relative; 
                    background: #f0f0f0;
                    margin: 20px 0;
                }
                .robot { 
                    width: 20px; 
                    height: 20px; 
                    background: blue; 
                    border-radius: 50%; 
                    position: absolute;
                    transition: all 2s ease-in-out;
                }
                .container-obj { 
                    width: 30px; 
                    height: 30px; 
                    background: green; 
                    position: absolute;
                }
                .pan-obj { 
                    width: 40px; 
                    height: 40px; 
                    background: silver; 
                    border-radius: 50%;
                    position: absolute;
                }
                .pod-obj { 
                    width: 15px; 
                    height: 15px; 
                    background: red; 
                    border-radius: 50%;
                    position: absolute;
                }
                .log { 
                    background: #000; 
                    color: #0f0; 
                    padding: 10px; 
                    height: 200px; 
                    overflow-y: scroll;
                    font-family: monospace;
                }
                .progress-bar {
                    width: 100%;
                    height: 20px;
                    background: #ddd;
                    margin: 10px 0;
                }
                .progress-fill {
                    height: 100%;
                    background: #4CAF50;
                    width: 0%;
                    transition: width 0.5s;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>🤖 Posha Robotics Assignment - Live Visualization</h1>
                <p>Watch the robot perform all assignment operations in real-time!</p>
                
                <div class="workspace" id="workspace">
                    <!-- Robot will be positioned here -->
                    <div class="robot" id="robot" style="left: 250px; top: 150px;"></div>
                    
                    <!-- Containers -->
                    <div class="container-obj" style="left: 400px; top: 100px;" title="Container 5"></div>
                    <div class="container-obj" style="left: 400px; top: 400px;" title="Container 1"></div>
                    
                    <!-- Pans -->
                    <div class="pan-obj" style="left: 200px; top: 400px;" title="Pan 1"></div>
                    <div class="pan-obj" style="left: 200px; top: 100px;" title="Pan 2"></div>
                    
                    <!-- Spice Pods -->
                    <div class="pod-obj" style="left: 100px; top: 150px;" title="Spice Pod 1"></div>
                    <div class="pod-obj" style="left: 100px; top: 350px;" title="Spice Pod 19"></div>
                </div>
                
                <div class="progress-bar">
                    <div class="progress-fill" id="progress"></div>
                </div>
                
                <div class="log" id="log">
                    <div>🚀 Starting Posha Robotics Simulation...</div>
                </div>
                
                <button onclick="startSimulation()">Start Simulation</button>
            </div>

            <script>
                const operations = [
                    { name: "Container 5 → Pan 2", start: [400, 100], end: [200, 100], type: "macro", time: 5 },
                    { name: "Container 1 → Pan 1", start: [400, 400], end: [200, 400], type: "macro", time: 5 },
                    { name: "Spice Pod 1 → Pan 2", start: [100, 150], end: [200, 100], type: "micro", time: 4 },
                    { name: "Spice Pod 19 → Pan 1", start: [100, 350], end: [200, 400], type: "micro", time: 4 }
                ];

                let currentOp = 0;
                const robot = document.getElementById('robot');
                const log = document.getElementById('log');
                const progress = document.getElementById('progress');

                function logMessage(message) {
                    const timestamp = new Date().toLocaleTimeString();
                    log.innerHTML += `<div>[${timestamp}] ${message}</div>`;
                    log.scrollTop = log.scrollHeight;
                }

                function moveRobot(startX, startY, endX, endY, duration, operationName) {
                    return new Promise((resolve) => {
                        robot.style.left = startX + 'px';
                        robot.style.top = startY + 'px';
                        
                        logMessage(`🤖 Starting: ${operationName}`);
                        
                        setTimeout(() => {
                            robot.style.left = endX + 'px';
                            robot.style.top = endY + 'px';
                            
                            setTimeout(() => {
                                logMessage(`✅ Completed: ${operationName}`);
                                resolve();
                            }, duration * 500);
                        }, 500);
                    });
                }

                async function startSimulation() {
                    logMessage("🎬 SIMULATION STARTED!");
                    
                    for (let i = 0; i < operations.length; i++) {
                        const op = operations[i];
                        progress.style.width = ((i / operations.length) * 100) + '%';
                        
                        await moveRobot(
                            op.start[0], op.start[1],
                            op.end[0], op.end[1],
                            op.time,
                            op.name
                        );
                        
                        // Return to center
                        await moveRobot(
                            op.end[0], op.end[1],
                            250, 150,
                            2,
                            "Returning to center"
                        );
                        
                        currentOp++;
                        progress.style.width = ((currentOp / operations.length) * 100) + '%';
                    }
                    
                    logMessage("🎉 ALL OPERATIONS COMPLETED SUCCESSFULLY!");
                    progress.style.width = '100%';
                    progress.style.background = '#4CAF50';
                }

                // Add labels
                const labels = [
                    { text: "Container 5", x: 420, y: 80 },
                    { text: "Container 1", x: 420, y: 430 },
                    { text: "Pan 2", x: 170, y: 80 },
                    { text: "Pan 1", x: 170, y: 430 },
                    { text: "Spice Pod 1", x: 50, y: 130 },
                    { text: "Spice Pod 19", x: 50, y: 370 },
                    { text: "🤖 Robot", x: 230, y: 170 }
                ];

                labels.forEach(label => {
                    const labelEl = document.createElement('div');
                    labelEl.textContent = label.text;
                    labelEl.style.position = 'absolute';
                    labelEl.style.left = label.x + 'px';
                    labelEl.style.top = label.y + 'px';
                    labelEl.style.fontSize = '12px';
                    labelEl.style.color = '#333';
                    document.getElementById('workspace').appendChild(labelEl);
                });
            </script>
        </body>
        </html>
        """
        
        # Save HTML file
        with open('posha_visualization.html', 'w') as f:
            f.write(html_content)
        
        print("🌐 Web visualization created: posha_visualization.html")
        print("📋 Opening in web browser...")
        
        # Open in default browser
        webbrowser.open('file://' + os.path.abspath('posha_visualization.html'))
        
        print("✅ Web visualization ready!")
        print("👆 Click 'Start Simulation' in the browser to see the robot move!")

def main():
    visualizer = WebVisualizer()
    visualizer.create_web_visualization()

if __name__ == "__main__":
    main()
