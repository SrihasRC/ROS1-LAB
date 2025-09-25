#!/usr/bin/env python3

"""
Enhanced Sensor Computation Node with Fixed Web Dashboard
Professional UI with proper chart sizing and no performance issues
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Float32
from geometry_msgs.msg import Vector3

import math
import numpy as np
from collections import deque
from datetime import datetime, timedelta
import json
import threading
import time
import webbrowser
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

class WebDashboardSensorCompute(Node):
    def __init__(self):
        super().__init__('web_dashboard_sensor_compute')
        
        # Declare parameters
        self.declare_parameter('log_interval', 10.0)
        self.declare_parameter('analysis_window', 60)
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('auto_open_browser', True)
        
        # Get parameters
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().double_value
        self.analysis_window = self.get_parameter('analysis_window').get_parameter_value().integer_value
        self.web_port = self.get_parameter('web_port').get_parameter_value().integer_value
        self.auto_open = self.get_parameter('auto_open_browser').get_parameter_value().bool_value
        
        # Data storage
        self.temp_history = deque(maxlen=self.analysis_window)
        self.humidity_history = deque(maxlen=self.analysis_window)
        self.heat_index_history = deque(maxlen=self.analysis_window)
        self.timestamp_history = deque(maxlen=self.analysis_window)
        
        # Current data for web dashboard
        self.current_data = {
            'temperature': 22.0,
            'humidity': 50.0,
            'heat_index': 22.0,
            'dew_point': 15.0,
            'comfort_level': 'Good',
            'air_quality': 'Optimal',
            'health_risk': 'No risks',
            'energy_advice': 'Efficient range',
            'trends': {'temperature': 'stable', 'humidity': 'stable'},
            'statistics': {
                'temp_avg': 22.0, 'temp_min': 22.0, 'temp_max': 22.0,
                'humidity_avg': 50.0, 'humidity_min': 50.0, 'humidity_max': 50.0,
                'messages_count': 0
            },
            'alerts': [],
            'last_update': datetime.now().isoformat(),
            'chart_data': {
                'timestamps': [],
                'temperatures': [],
                'humidities': [],
                'heat_indices': []
            }
        }
        
        # State tracking
        self.last_log_time = 0
        self.message_count = 0
        self.last_alert_time = {}
        
        # Setup ROS2
        self.setup_ros2()
        
        # Start web server
        self.start_web_server()
        
        self.get_logger().info(f'Sensor compute with web server started')
        self.get_logger().info(f'Dashboard: http://localhost:{self.web_port}')
        
        if self.auto_open:
            threading.Timer(2.0, lambda: webbrowser.open(f'http://localhost:{self.web_port}')).start()
    
    def setup_ros2(self):
        """Setup ROS2 subscribers and publishers"""
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.compute_callback,
            10
        )
        
        # Publishers
        self.heat_index_pub = self.create_publisher(Float32, 'computed/heat_index', 10)
        self.comfort_level_pub = self.create_publisher(String, 'computed/comfort_level', 10)
        self.alert_pub = self.create_publisher(String, 'computed/alerts', 10)
        
        # Timer for periodic updates
        self.analysis_timer = self.create_timer(2.0, self.update_web_data)
    
    def compute_callback(self, msg):
        """Process incoming sensor data"""
        if len(msg.data) < 2:
            return
        
        temp, humidity = msg.data[0], msg.data[1]
        current_time = time.time()
        
        # Store data
        self.temp_history.append(temp)
        self.humidity_history.append(humidity)
        self.timestamp_history.append(current_time)
        self.message_count += 1
        
        # Perform computations
        heat_index = self.calculate_heat_index(temp, humidity)
        dew_point = self.calculate_dew_point(temp, humidity)
        comfort_level = self.assess_comfort_level(temp, humidity)
        air_quality = self.assess_air_quality(humidity)
        health_risk = self.assess_health_risk(temp, humidity, heat_index)
        energy_advice = self.generate_energy_advice(temp, humidity)
        
        # Store heat index
        self.heat_index_history.append(heat_index)
        
        # Update current data
        self.current_data.update({
            'temperature': round(temp, 1),
            'humidity': round(humidity, 1),
            'heat_index': round(heat_index, 1),
            'dew_point': round(dew_point, 1),
            'comfort_level': comfort_level,
            'air_quality': air_quality,
            'health_risk': health_risk,
            'energy_advice': energy_advice,
            'last_update': datetime.now().isoformat()
        })
        
        # Publish ROS2 messages
        heat_msg = Float32()
        heat_msg.data = float(heat_index)
        self.heat_index_pub.publish(heat_msg)
        
        comfort_msg = String()
        comfort_msg.data = comfort_level
        self.comfort_level_pub.publish(comfort_msg)
        
        # Check for alerts
        self.check_alerts(temp, humidity, heat_index)
        
        # Log periodically
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(
                f"Data: T:{temp:.1f}°C H:{humidity:.1f}% HI:{heat_index:.1f}°C "
                f"Comfort:{comfort_level} ({self.message_count} msgs)"
            )
            self.last_log_time = current_time
    
    def update_web_data(self):
        """Update data for web dashboard"""
        if len(self.temp_history) < 2:
            return
        
        # Update trends
        if len(self.temp_history) >= 10:
            temp_trend = self.calculate_simple_trend(list(self.temp_history)[-10:])
            humidity_trend = self.calculate_simple_trend(list(self.humidity_history)[-10:])
            
            self.current_data['trends'] = {
                'temperature': 'rising' if temp_trend > 0.1 else 'falling' if temp_trend < -0.1 else 'stable',
                'humidity': 'rising' if humidity_trend > 1.0 else 'falling' if humidity_trend < -1.0 else 'stable'
            }
        
        # Update statistics
        if self.temp_history:
            temp_data = list(self.temp_history)
            humidity_data = list(self.humidity_history)
            
            self.current_data['statistics'] = {
                'temp_avg': round(np.mean(temp_data), 1),
                'temp_min': round(np.min(temp_data), 1),
                'temp_max': round(np.max(temp_data), 1),
                'humidity_avg': round(np.mean(humidity_data), 1),
                'humidity_min': round(np.min(humidity_data), 1),
                'humidity_max': round(np.max(humidity_data), 1),
                'messages_count': self.message_count
            }
        
        # Update chart data (last 20 points for performance)
        if len(self.temp_history) > 0:
            recent_count = min(20, len(self.temp_history))
            timestamps = [datetime.fromtimestamp(t).strftime('%H:%M:%S') 
                         for t in list(self.timestamp_history)[-recent_count:]]
            
            self.current_data['chart_data'] = {
                'timestamps': timestamps,
                'temperatures': list(self.temp_history)[-recent_count:],
                'humidities': list(self.humidity_history)[-recent_count:],
                'heat_indices': list(self.heat_index_history)[-recent_count:] if self.heat_index_history else []
            }
    
    def calculate_heat_index(self, temp_c, humidity):
        """Calculate heat index"""
        temp_f = temp_c * 9/5 + 32
        if temp_f < 80:
            return temp_c
        
        # Simplified heat index calculation
        c1, c2, c3 = -42.379, 2.04901523, 10.14333127
        c4, c5, c6 = -0.22475541, -6.83783e-3, -5.481717e-2
        
        hi_f = c1 + c2*temp_f + c3*humidity + c4*temp_f*humidity + c5*temp_f**2 + c6*humidity**2
        return (hi_f - 32) * 5/9
    
    def calculate_dew_point(self, temp_c, humidity):
        """Calculate dew point"""
        a, b = 17.27, 237.7
        alpha = ((a * temp_c) / (b + temp_c)) + math.log(humidity / 100.0)
        return (b * alpha) / (a - alpha)
    
    def assess_comfort_level(self, temp, humidity):
        """Assess comfort level"""
        if 20 <= temp <= 26 and 40 <= humidity <= 60:
            return "Excellent"
        elif 18 <= temp <= 28 and 30 <= humidity <= 70:
            return "Good"
        elif 16 <= temp <= 30 and 25 <= humidity <= 75:
            return "Acceptable"
        else:
            return "Uncomfortable"
    
    def assess_air_quality(self, humidity):
        """Assess air quality based on humidity"""
        if 40 <= humidity <= 60:
            return "Optimal"
        elif 30 <= humidity <= 70:
            return "Good"
        elif humidity < 30:
            return "Too Dry"
        else:
            return "Too Humid"
    
    def assess_health_risk(self, temp, humidity, heat_index):
        """Assess health risks"""
        risks = []
        if heat_index > 32:
            risks.append("Heat stress")
        if humidity > 80:
            risks.append("Mold risk")
        if humidity < 20:
            risks.append("Dry air irritation")
        
        return "; ".join(risks) if risks else "No significant risks"
    
    def generate_energy_advice(self, temp, humidity):
        """Generate energy advice"""
        if temp > 26:
            return "Consider cooling system"
        elif temp < 20:
            return "Consider heating system"
        elif 20 <= temp <= 24 and 40 <= humidity <= 60:
            return "Optimal efficiency range"
        else:
            return "Moderate energy usage"
    
    def calculate_simple_trend(self, data):
        """Calculate simple trend slope"""
        if len(data) < 2:
            return 0
        n = len(data)
        x = np.arange(n)
        return np.polyfit(x, data, 1)[0]
    
    def check_alerts(self, temp, humidity, heat_index):
        """Check for alert conditions"""
        current_time = time.time()
        new_alerts = []
        
        if temp > 35 and self.should_alert('extreme_heat', current_time):
            new_alerts.append({"type": "danger", "message": "Extreme heat detected"})
        
        if humidity > 85 and self.should_alert('high_humidity', current_time):
            new_alerts.append({"type": "warning", "message": "Very high humidity"})
        
        if heat_index > 40 and self.should_alert('dangerous_heat_index', current_time):
            new_alerts.append({"type": "danger", "message": "Dangerous heat index"})
        
        # Add new alerts and keep only recent ones
        self.current_data['alerts'].extend(new_alerts)
        self.current_data['alerts'] = self.current_data['alerts'][-5:]  # Keep last 5 alerts
        
        # Publish alerts to ROS
        for alert in new_alerts:
            alert_msg = String()
            alert_msg.data = alert['message']
            self.alert_pub.publish(alert_msg)
    
    def should_alert(self, alert_type, current_time):
        """Check if alert should be sent (rate limiting)"""
        last_alert = self.last_alert_time.get(alert_type, 0)
        if current_time - last_alert > 60:  # 1 minute between same alerts
            self.last_alert_time[alert_type] = current_time
            return True
        return False
    
    def start_web_server(self):
        """Start built-in HTTP server"""
        server_instance = self  # Reference to access data
        
        class DashboardHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass  # Suppress HTTP server logs
            
            def do_GET(self):
                if self.path == '/':
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    self.wfile.write(self.get_dashboard_html().encode())
                
                elif self.path == '/api/data':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(json.dumps(server_instance.current_data).encode())
                
                else:
                    self.send_response(404)
                    self.end_headers()
            
            def get_dashboard_html(self):
                return '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROS2 Sensor Dashboard</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        .chart-container {
            position: relative;
            height: 300px;
            width: 100%;
        }
        
        .metric-card {
            transition: all 0.3s ease;
        }
        
        .metric-card:hover {
            transform: translateY(-2px);
        }
        
        .status-indicator {
            animation: pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite;
        }
    </style>
</head>
<body class="bg-gray-50 min-h-screen">
    <!-- Header -->
    <nav class="bg-white shadow-sm border-b border-gray-200">
        <div class="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div class="flex justify-between items-center h-16">
                <div class="flex items-center">
                    <h1 class="text-2xl font-semibold text-gray-900">Sensor Dashboard</h1>
                    <span class="ml-3 text-sm text-gray-500">ROS2 Environmental Monitor</span>
                </div>
                <div class="flex items-center space-x-4">
                    <div id="status" class="flex items-center space-x-2">
                        <div class="w-3 h-3 bg-green-500 rounded-full status-indicator"></div>
                        <span class="text-sm font-medium text-gray-700">Live</span>
                    </div>
                    <div class="text-sm text-gray-500">
                        <span id="lastUpdate">--:--:--</span>
                    </div>
                </div>
            </div>
        </div>
    </nav>

    <div class="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        <!-- Current Readings -->
        <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
            <div class="metric-card bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <div class="flex items-center justify-between">
                    <div>
                        <p class="text-sm font-medium text-gray-600">Temperature</p>
                        <p id="temperature" class="text-2xl font-bold text-red-600">--°C</p>
                    </div>
                    <div class="w-12 h-12 bg-red-100 rounded-full flex items-center justify-center">
                        <svg class="w-6 h-6 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 10V3L4 14h7v7l9-11h-7z"></path>
                        </svg>
                    </div>
                </div>
                <div class="mt-4">
                    <span id="tempTrend" class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800">--</span>
                </div>
            </div>

            <div class="metric-card bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <div class="flex items-center justify-between">
                    <div>
                        <p class="text-sm font-medium text-gray-600">Humidity</p>
                        <p id="humidity" class="text-2xl font-bold text-blue-600">--%</p>
                    </div>
                    <div class="w-12 h-12 bg-blue-100 rounded-full flex items-center justify-center">
                        <svg class="w-6 h-6 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M3 15a4 4 0 004 4h9a5 5 0 10-.1-9.999 5.002 5.002 0 10-9.78 2.096A4.001 4.001 0 003 15z"></path>
                        </svg>
                    </div>
                </div>
                <div class="mt-4">
                    <span id="humidityTrend" class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800">--</span>
                </div>
            </div>

            <div class="metric-card bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <div class="flex items-center justify-between">
                    <div>
                        <p class="text-sm font-medium text-gray-600">Heat Index</p>
                        <p id="heatIndex" class="text-2xl font-bold text-orange-600">--°C</p>
                    </div>
                    <div class="w-12 h-12 bg-orange-100 rounded-full flex items-center justify-center">
                        <svg class="w-6 h-6 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 3v1m0 16v1m9-9h-1M4 12H3m15.364 6.364l-.707-.707M6.343 6.343l-.707-.707m12.728 0l-.707.707M6.343 17.657l-.707.707M16 12a4 4 0 11-8 0 4 4 0 018 0z"></path>
                        </svg>
                    </div>
                </div>
                <div class="mt-4">
                    <span id="comfortLevel" class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">--</span>
                </div>
            </div>

            <div class="metric-card bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <div class="flex items-center justify-between">
                    <div>
                        <p class="text-sm font-medium text-gray-600">Dew Point</p>
                        <p id="dewPoint" class="text-2xl font-bold text-teal-600">--°C</p>
                    </div>
                    <div class="w-12 h-12 bg-teal-100 rounded-full flex items-center justify-center">
                        <svg class="w-6 h-6 text-teal-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M7 21a4 4 0 01-4-4V5a2 2 0 012-2h4a2 2 0 012 2v12a4 4 0 01-4 4zM7 3V1m0 18v2m-4-9h8"></path>
                        </svg>
                    </div>
                </div>
                <div class="mt-4">
                    <span id="airQuality" class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800">--</span>
                </div>
            </div>
        </div>

        <!-- Charts and Analysis -->
        <div class="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
            <!-- Chart -->
            <div class="lg:col-span-2 bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <div class="flex items-center justify-between mb-6">
                    <h3 class="text-lg font-semibold text-gray-900">Real-time Trends</h3>
                    <div class="text-sm text-gray-500">Last 20 readings</div>
                </div>
                <div class="chart-container">
                    <canvas id="sensorChart"></canvas>
                </div>
            </div>

            <!-- Statistics -->
            <div class="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <h3 class="text-lg font-semibold text-gray-900 mb-6">Statistics</h3>
                <div class="space-y-6">
                    <div>
                        <div class="flex items-center justify-between text-sm">
                            <span class="text-gray-600">Avg Temperature</span>
                            <span id="tempAvg" class="font-semibold text-gray-900">--°C</span>
                        </div>
                        <div class="flex items-center justify-between text-sm mt-1">
                            <span class="text-gray-600">Range</span>
                            <span class="font-semibold text-gray-900">
                                <span id="tempMin">--</span> to <span id="tempMax">--</span>°C
                            </span>
                        </div>
                    </div>
                    
                    <div class="border-t pt-4">
                        <div class="flex items-center justify-between text-sm">
                            <span class="text-gray-600">Avg Humidity</span>
                            <span id="humidityAvg" class="font-semibold text-gray-900">--%</span>
                        </div>
                        <div class="flex items-center justify-between text-sm mt-1">
                            <span class="text-gray-600">Range</span>
                            <span class="font-semibold text-gray-900">
                                <span id="humidityMin">--</span> to <span id="humidityMax">--</span>%
                            </span>
                        </div>
                    </div>
                    
                    <div class="border-t pt-4">
                        <div class="flex items-center justify-between text-sm">
                            <span class="text-gray-600">Total Messages</span>
                            <span id="messagesCount" class="font-semibold text-gray-900">--</span>
                        </div>
                        <div class="flex items-center justify-between text-sm mt-1">
                            <span class="text-gray-600">Data Points</span>
                            <span id="dataPoints" class="font-semibold text-gray-900">--</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Analysis Cards -->
        <div class="grid grid-cols-1 md:grid-cols-3 gap-6">
            <div class="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <h3 class="text-lg font-semibold text-gray-900 mb-4">Health Assessment</h3>
                <p id="healthRisk" class="text-gray-700 text-sm leading-relaxed">--</p>
            </div>

            <div class="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <h3 class="text-lg font-semibold text-gray-900 mb-4">Energy Advice</h3>
                <p id="energyAdvice" class="text-gray-700 text-sm leading-relaxed">--</p>
            </div>

            <div class="bg-white rounded-lg shadow-sm border border-gray-200 p-6">
                <h3 class="text-lg font-semibold text-gray-900 mb-4">System Alerts</h3>
                <div id="alerts" class="space-y-2">
                    <p class="text-gray-500 text-sm">No active alerts</p>
                </div>
            </div>
        </div>
    </div>

    <script>
        let chart;
        
        function initChart() {
            const ctx = document.getElementById('sensorChart').getContext('2d');
            chart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'Temperature (°C)',
                            data: [],
                            borderColor: 'rgb(220, 38, 38)',
                            backgroundColor: 'rgba(220, 38, 38, 0.1)',
                            borderWidth: 2,
                            tension: 0.4,
                            fill: false
                        },
                        {
                            label: 'Humidity (%)',
                            data: [],
                            borderColor: 'rgb(37, 99, 235)',
                            backgroundColor: 'rgba(37, 99, 235, 0.1)',
                            borderWidth: 2,
                            tension: 0.4,
                            fill: false
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    interaction: {
                        intersect: false,
                    },
                    plugins: {
                        legend: {
                            position: 'top',
                        }
                    },
                    scales: {
                        x: {
                            display: true,
                            grid: {
                                display: false
                            }
                        },
                        y: {
                            display: true,
                            beginAtZero: false,
                            grid: {
                                color: 'rgba(0, 0, 0, 0.1)'
                            }
                        }
                    },
                    elements: {
                        point: {
                            radius: 3,
                            hoverRadius: 5
                        }
                    },
                    animation: {
                        duration: 0 // Disable animations for better performance
                    }
                }
            });
        }
        
        function updateDashboard() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    // Update main metrics
                    document.getElementById('temperature').textContent = data.temperature + '°C';
                    document.getElementById('humidity').textContent = data.humidity + '%';
                    document.getElementById('heatIndex').textContent = data.heat_index + '°C';
                    document.getElementById('dewPoint').textContent = data.dew_point + '°C';
                    
                    // Update trends with proper styling
                    updateTrendBadge('tempTrend', data.trends.temperature);
                    updateTrendBadge('humidityTrend', data.trends.humidity);
                    
                    // Update status badges
                    updateComfortBadge('comfortLevel', data.comfort_level);
                    updateAirQualityBadge('airQuality', data.air_quality);
                    
                    // Update analysis sections
                    document.getElementById('healthRisk').textContent = data.health_risk;
                    document.getElementById('energyAdvice').textContent = data.energy_advice;
                    
                    // Update statistics
                    updateStatistics(data.statistics);
                    
                    // Update chart with fixed size
                    updateChart(data.chart_data);
                    
                    // Update alerts
                    updateAlerts(data.alerts);
                    
                    // Update timestamp
                    const lastUpdate = new Date(data.last_update).toLocaleTimeString();
                    document.getElementById('lastUpdate').textContent = lastUpdate;
                    
                    // Update connection status
                    document.getElementById('status').innerHTML = 
                        '<div class="w-3 h-3 bg-green-500 rounded-full status-indicator"></div>' +
                        '<span class="text-sm font-medium text-gray-700">Live</span>';
                })
                .catch(error => {
                    console.error('Error fetching data:', error);
                    document.getElementById('status').innerHTML = 
                        '<div class="w-3 h-3 bg-red-500 rounded-full"></div>' +
                        '<span class="text-sm font-medium text-gray-700">Error</span>';
                });
        }
        
        function updateTrendBadge(elementId, trend) {
            const element = document.getElementById(elementId);
            element.textContent = trend.charAt(0).toUpperCase() + trend.slice(1);
            
            // Update badge styling based on trend
            if (trend === 'rising') {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-red-100 text-red-800';
            } else if (trend === 'falling') {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800';
            } else {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800';
            }
        }
        
        function updateComfortBadge(elementId, comfort) {
            const element = document.getElementById(elementId);
            element.textContent = comfort;
            
            if (comfort === 'Excellent') {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800';
            } else if (comfort === 'Good') {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800';
            } else if (comfort === 'Acceptable') {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800';
            } else {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-red-100 text-red-800';
            }
        }
        
        function updateAirQualityBadge(elementId, quality) {
            const element = document.getElementById(elementId);
            element.textContent = quality;
            
            if (quality === 'Optimal') {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800';
            } else if (quality === 'Good') {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800';
            } else {
                element.className = 'inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800';
            }
        }
        
        function updateStatistics(stats) {
            document.getElementById('tempAvg').textContent = stats.temp_avg + '°C';
            document.getElementById('tempMin').textContent = stats.temp_min;
            document.getElementById('tempMax').textContent = stats.temp_max;
            document.getElementById('humidityAvg').textContent = stats.humidity_avg + '%';
            document.getElementById('humidityMin').textContent = stats.humidity_min;
            document.getElementById('humidityMax').textContent = stats.humidity_max;
            document.getElementById('messagesCount').textContent = stats.messages_count;
            document.getElementById('dataPoints').textContent = stats.messages_count;
        }
        
        function updateChart(chartData) {
            if (!chart || chartData.timestamps.length === 0) {
                return;
            }
            
            // Update chart data with fixed dataset length (max 20 points)
            chart.data.labels = chartData.timestamps;
            chart.data.datasets[0].data = chartData.temperatures;
            chart.data.datasets[1].data = chartData.humidities;
            
            // Use 'none' mode to prevent animations during updates for better performance
            chart.update('none');
        }
        
        function updateAlerts(alerts) {
            const alertsContainer = document.getElementById('alerts');
            
            if (alerts.length === 0) {
                alertsContainer.innerHTML = '<p class="text-gray-500 text-sm">No active alerts</p>';
                return;
            }
            
            alertsContainer.innerHTML = alerts.map(alert => {
                const bgColor = alert.type === 'danger' ? 'bg-red-50 text-red-800 border-red-200' : 'bg-yellow-50 text-yellow-800 border-yellow-200';
                return `<div class="p-3 rounded-lg border text-sm ${bgColor}">
                    ${alert.message}
                </div>`;
            }).join('');
        }
        
        // Initialize dashboard
        document.addEventListener('DOMContentLoaded', function() {
            initChart();
            updateDashboard();
            
            // Update every 2 seconds
            setInterval(updateDashboard, 2000);
        });
    </script>
</body>
</html>'''
        
        # Start server in separate thread
        def run_server():
            server = HTTPServer(('localhost', self.web_port), DashboardHandler)
            server.serve_forever()
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

def main(args=None):
    rclpy.init(args=args)
    node = WebDashboardSensorCompute()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down sensor compute with web dashboard...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
