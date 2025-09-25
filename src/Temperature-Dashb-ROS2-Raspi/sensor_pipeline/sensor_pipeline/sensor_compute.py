#!/usr/bin/env python3

"""
Enhanced Sensor Computation Node for Robot Programming Assignment
Performs advanced calculations, trend analysis, and intelligent alerting
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Float32, Bool
from geometry_msgs.msg import Vector3

import math
import numpy as np
from collections import deque
from datetime import datetime, timedelta
import json

class EnhancedSensorCompute(Node):
    def __init__(self):
        super().__init__('enhanced_sensor_compute')
        
        # Declare parameters
        self.declare_parameter('log_interval', 10.0)  # Log every 10 seconds
        self.declare_parameter('analysis_window', 60)  # 60 data points for analysis
        self.declare_parameter('alert_thresholds', True)  # Enable alerting
        self.declare_parameter('comfort_zones', True)  # Enable comfort analysis
        
        # Get parameters
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().double_value
        self.analysis_window = self.get_parameter('analysis_window').get_parameter_value().integer_value
        self.alert_enabled = self.get_parameter('alert_thresholds').get_parameter_value().bool_value
        self.comfort_enabled = self.get_parameter('comfort_zones').get_parameter_value().bool_value
        
        # Data storage for analysis
        self.temp_history = deque(maxlen=self.analysis_window)
        self.humidity_history = deque(maxlen=self.analysis_window)
        self.heat_index_history = deque(maxlen=self.analysis_window)
        self.timestamp_history = deque(maxlen=self.analysis_window)
        
        # State tracking
        self.last_log_time = 0
        self.message_count = 0
        self.last_alert_time = {}  # Track last alert for each type
        self.trend_direction = {"temperature": "stable", "humidity": "stable"}
        
        # Comfort zone definitions (based on ASHRAE standards)
        self.comfort_zones = {
            "winter": {"temp_range": (20, 24), "humidity_range": (30, 60)},
            "summer": {"temp_range": (23, 26), "humidity_range": (30, 60)},
            "optimal": {"temp_range": (22, 25), "humidity_range": (40, 60)}
        }
        
        # Subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.compute_callback,
            10
        )
        
        # Publishers for computed data
        self.heat_index_pub = self.create_publisher(Float32, 'computed/heat_index', 10)
        self.dew_point_pub = self.create_publisher(Float32, 'computed/dew_point', 10)
        self.comfort_level_pub = self.create_publisher(String, 'computed/comfort_level', 10)
        self.trend_pub = self.create_publisher(Vector3, 'computed/trends', 10)
        self.alert_pub = self.create_publisher(String, 'computed/alerts', 10)
        self.air_quality_pub = self.create_publisher(String, 'computed/air_quality', 10)
        self.energy_advice_pub = self.create_publisher(String, 'computed/energy_advice', 10)
        self.health_risk_pub = self.create_publisher(String, 'computed/health_risk', 10)
        self.prediction_pub = self.create_publisher(Vector3, 'computed/predictions', 10)
        
        # Statistics publisher
        self.stats_pub = self.create_publisher(String, 'computed/statistics', 10)
        
        # Timer for periodic analysis
        self.analysis_timer = self.create_timer(30.0, self.perform_advanced_analysis)
        
        self.get_logger().info(f'Enhanced sensor compute node started')
        self.get_logger().info(f'Logging interval: {self.log_interval}s, Analysis window: {self.analysis_window} points')
    
    def compute_callback(self, msg):
        """Main callback for processing sensor data"""
        if len(msg.data) < 2:
            self.get_logger().warning("Invalid sensor data format")
            return
        
        temp, humidity = msg.data[0], msg.data[1]
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Store data for analysis
        self.temp_history.append(temp)
        self.humidity_history.append(humidity)
        self.timestamp_history.append(current_time)
        
        self.message_count += 1
        
        # Perform all computations
        computed_results = self.perform_computations(temp, humidity)
        
        # Publish computed values
        self.publish_computed_data(computed_results)
        
        # Check for alerts
        if self.alert_enabled:
            self.check_alerts(computed_results)
        
        # Log periodically (not every message)
        self.periodic_logging(computed_results, current_time)
    
    def perform_computations(self, temp, humidity):
        """Perform all sensor computations"""
        results = {
            'temperature': temp,
            'humidity': humidity,
            'timestamp': datetime.now().isoformat()
        }
        
        # 1. Heat Index (Apparent Temperature)
        results['heat_index'] = self.calculate_heat_index(temp, humidity)
        
        # 2. Dew Point
        results['dew_point'] = self.calculate_dew_point(temp, humidity)
        
        # 3. Comfort Level Assessment
        results['comfort_level'] = self.assess_comfort_level(temp, humidity)
        
        # 4. Air Quality Index (humidity-based)
        results['air_quality'] = self.assess_air_quality(temp, humidity)
        
        # 5. Health Risk Assessment
        results['health_risk'] = self.assess_health_risk(temp, humidity, results['heat_index'])
        
        # 6. Energy Efficiency Advice
        results['energy_advice'] = self.generate_energy_advice(temp, humidity)
        
        # 7. Trend Analysis (if enough data)
        if len(self.temp_history) >= 10:
            results['trends'] = self.analyze_trends()
        
        # 8. Simple Prediction (if enough data)
        if len(self.temp_history) >= 20:
            results['predictions'] = self.simple_prediction()
        
        # Store heat index for history
        self.heat_index_history.append(results['heat_index'])
        
        return results
    
    def calculate_heat_index(self, temp_c, humidity):
        """Calculate heat index using advanced Rothfusz equation"""
        # Convert to Fahrenheit
        temp_f = temp_c * 9/5 + 32
        
        if temp_f < 80:
            return temp_c  # Heat index not meaningful below 26.7°C
        
        # Rothfusz equation coefficients
        c = [-42.379, 2.04901523, 10.14333127, -0.22475541, -6.83783e-3,
             -5.481717e-2, 1.22874e-3, 8.5282e-4, -1.99e-6]
        
        rh = humidity
        t = temp_f
        
        # Calculate heat index in Fahrenheit
        hi = (c[0] + c[1]*t + c[2]*rh + c[3]*t*rh + c[4]*t*t + 
              c[5]*rh*rh + c[6]*t*t*rh + c[7]*t*rh*rh + c[8]*t*t*rh*rh)
        
        # Apply corrections for extreme conditions
        if rh < 13 and 80 <= temp_f <= 112:
            hi -= ((13 - rh) / 4) * math.sqrt((17 - abs(temp_f - 95)) / 17)
        elif rh > 85 and 80 <= temp_f <= 87:
            hi += ((rh - 85) / 10) * ((87 - temp_f) / 5)
        
        # Convert back to Celsius
        return (hi - 32) * 5/9
    
    def calculate_dew_point(self, temp_c, humidity):
        """Calculate dew point using Magnus formula"""
        a, b = 17.27, 237.7
        alpha = ((a * temp_c) / (b + temp_c)) + math.log(humidity / 100.0)
        return (b * alpha) / (a - alpha)
    
    def assess_comfort_level(self, temp, humidity):
        """Assess human comfort level based on temperature and humidity"""
        # Multi-tier comfort assessment
        comfort_scores = []
        
        # Check against different comfort zones
        for zone_name, zone in self.comfort_zones.items():
            temp_score = self.calculate_range_score(temp, zone["temp_range"])
            humidity_score = self.calculate_range_score(humidity, zone["humidity_range"])
            combined_score = (temp_score + humidity_score) / 2
            comfort_scores.append((zone_name, combined_score))
        
        # Find best matching zone
        best_zone, best_score = max(comfort_scores, key=lambda x: x[1])
        
        if best_score >= 0.8:
            return f"Excellent ({best_zone})"
        elif best_score >= 0.6:
            return f"Good ({best_zone})"
        elif best_score >= 0.4:
            return f"Acceptable ({best_zone})"
        elif best_score >= 0.2:
            return "Uncomfortable"
        else:
            return "Very Uncomfortable"
    
    def calculate_range_score(self, value, optimal_range):
        """Calculate how well a value fits within an optimal range"""
        min_val, max_val = optimal_range
        if min_val <= value <= max_val:
            return 1.0  # Perfect score within range
        elif value < min_val:
            # Score decreases as we go further below the range
            distance = min_val - value
            return max(0.0, 1.0 - distance / min_val)
        else:
            # Score decreases as we go further above the range
            distance = value - max_val
            return max(0.0, 1.0 - distance / max_val)
    
    def assess_air_quality(self, temp, humidity):
        """Assess air quality based on humidity levels"""
        if humidity < 30:
            return "Too Dry - Risk of respiratory irritation"
        elif humidity > 70:
            return "Too Humid - Risk of mold growth"
        elif 40 <= humidity <= 60:
            return "Optimal - Good for health"
        elif 30 <= humidity < 40 or 60 < humidity <= 70:
            return "Acceptable - Minor concerns"
        else:
            return "Monitor closely"
    
    def assess_health_risk(self, temp, humidity, heat_index):
        """Assess health risks based on environmental conditions"""
        risks = []
        
        # Heat-related risks
        if heat_index > 32:
            risks.append("Heat exhaustion risk")
        elif heat_index > 27:
            risks.append("Fatigue with prolonged exposure")
        
        # Humidity-related risks
        if humidity > 80:
            risks.append("Mold/bacteria growth risk")
        elif humidity < 20:
            risks.append("Respiratory irritation risk")
        
        # Temperature extremes
        if temp > 30:
            risks.append("Overheating risk")
        elif temp < 16:
            risks.append("Cold stress risk")
        
        if not risks:
            return "No significant health risks"
        else:
            return "; ".join(risks)
    
    def generate_energy_advice(self, temp, humidity):
        """Generate energy efficiency advice"""
        advice = []
        
        # Heating/cooling advice
        if temp > 26:
            advice.append("Consider cooling - each degree reduction saves ~8% energy")
        elif temp < 20:
            advice.append("Consider heating - maintain 20-22°C for efficiency")
        
        # Humidity advice
        if humidity > 60:
            advice.append("Use dehumidifier - high humidity makes it feel warmer")
        elif humidity < 40:
            advice.append("Consider humidifier - low humidity makes it feel cooler")
        
        # Optimal advice
        if 20 <= temp <= 24 and 40 <= humidity <= 60:
            advice.append("Optimal conditions - energy efficient range")
        
        return "; ".join(advice) if advice else "Current conditions are energy efficient"
    
    def analyze_trends(self):
        """Analyze trends in temperature and humidity"""
        if len(self.temp_history) < 10:
            return {"temperature": 0, "humidity": 0, "heat_index": 0}
        
        # Calculate trends using linear regression
        temp_trend = self.calculate_trend(list(self.temp_history))
        humidity_trend = self.calculate_trend(list(self.humidity_history))
        heat_index_trend = self.calculate_trend(list(self.heat_index_history))
        
        # Update trend direction
        self.trend_direction["temperature"] = "rising" if temp_trend > 0.05 else "falling" if temp_trend < -0.05 else "stable"
        self.trend_direction["humidity"] = "rising" if humidity_trend > 0.5 else "falling" if humidity_trend < -0.5 else "stable"
        
        return {
            "temperature": temp_trend,
            "humidity": humidity_trend, 
            "heat_index": heat_index_trend
        }
    
    def calculate_trend(self, data):
        """Calculate trend using simple linear regression"""
        n = len(data)
        x = np.arange(n)
        y = np.array(data)
        
        # Calculate slope
        slope = (n * np.sum(x * y) - np.sum(x) * np.sum(y)) / (n * np.sum(x**2) - np.sum(x)**2)
        return slope
    
    def simple_prediction(self):
        """Simple prediction based on trends"""
        if len(self.temp_history) < 20:
            return {"temperature": 0, "humidity": 0, "confidence": 0}
        
        # Use recent trend to predict next few minutes
        recent_temp = list(self.temp_history)[-10:]
        recent_humidity = list(self.humidity_history)[-10:]
        
        temp_trend = self.calculate_trend(recent_temp)
        humidity_trend = self.calculate_trend(recent_humidity)
        
        # Predict 5 minutes ahead (assuming 2-second intervals = 150 data points)
        temp_prediction = recent_temp[-1] + (temp_trend * 150)
        humidity_prediction = recent_humidity[-1] + (humidity_trend * 150)
        
        # Calculate confidence based on trend stability
        confidence = min(1.0, 1.0 / (1.0 + abs(temp_trend) + abs(humidity_trend)))
        
        return {
            "temperature": temp_prediction,
            "humidity": humidity_prediction,
            "confidence": confidence
        }
    
    def check_alerts(self, results):
        """Check for alert conditions"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        alerts = []
        
        # Temperature alerts
        if results['temperature'] > 35:
            if self.should_send_alert('extreme_heat', current_time):
                alerts.append("🔥 EXTREME HEAT ALERT: Temperature above 35°C!")
        elif results['temperature'] < 10:
            if self.should_send_alert('extreme_cold', current_time):
                alerts.append("🧊 EXTREME COLD ALERT: Temperature below 10°C!")
        
        # Humidity alerts
        if results['humidity'] > 85:
            if self.should_send_alert('high_humidity', current_time):
                alerts.append("💧 HIGH HUMIDITY ALERT: Risk of mold growth!")
        elif results['humidity'] < 20:
            if self.should_send_alert('low_humidity', current_time):
                alerts.append("🏜️ LOW HUMIDITY ALERT: Air too dry!")
        
        # Heat index alerts
        if results['heat_index'] > 40:
            if self.should_send_alert('dangerous_heat_index', current_time):
                alerts.append("⚠️ DANGEROUS CONDITIONS: Heat index above 40°C!")
        
        # Trend alerts
        if hasattr(results, 'trends'):
            if results['trends']['temperature'] > 0.5:  # Rising fast
                if self.should_send_alert('rapid_temp_rise', current_time):
                    alerts.append("📈 RAPID TEMPERATURE RISE detected!")
        
        # Publish alerts
        for alert in alerts:
            alert_msg = String()
            alert_msg.data = alert
            self.alert_pub.publish(alert_msg)
            self.get_logger().warn(alert)
    
    def should_send_alert(self, alert_type, current_time):
        """Check if enough time has passed since last alert of this type"""
        last_alert = self.last_alert_time.get(alert_type, 0)
        if current_time - last_alert > 300:  # 5 minutes between same alerts
            self.last_alert_time[alert_type] = current_time
            return True
        return False
    
    def publish_computed_data(self, results):
        """Publish all computed data to respective topics"""
        # Heat index
        heat_msg = Float32()
        heat_msg.data = float(results['heat_index'])
        self.heat_index_pub.publish(heat_msg)
        
        # Dew point
        dew_msg = Float32()
        dew_msg.data = float(results['dew_point'])
        self.dew_point_pub.publish(dew_msg)
        
        # Comfort level
        comfort_msg = String()
        comfort_msg.data = results['comfort_level']
        self.comfort_level_pub.publish(comfort_msg)
        
        # Air quality
        air_msg = String()
        air_msg.data = results['air_quality']
        self.air_quality_pub.publish(air_msg)
        
        # Energy advice
        energy_msg = String()
        energy_msg.data = results['energy_advice']
        self.energy_advice_pub.publish(energy_msg)
        
        # Health risk
        health_msg = String()
        health_msg.data = results['health_risk']
        self.health_risk_pub.publish(health_msg)
        
        # Trends (if available)
        if 'trends' in results:
            trend_msg = Vector3()
            trend_msg.x = float(results['trends']['temperature'])
            trend_msg.y = float(results['trends']['humidity'])
            trend_msg.z = float(results['trends']['heat_index'])
            self.trend_pub.publish(trend_msg)
        
        # Predictions (if available)
        if 'predictions' in results:
            pred_msg = Vector3()
            pred_msg.x = float(results['predictions']['temperature'])
            pred_msg.y = float(results['predictions']['humidity'])
            pred_msg.z = float(results['predictions']['confidence'])
            self.prediction_pub.publish(pred_msg)
    
    def periodic_logging(self, results, current_time):
        """Log comprehensive information periodically"""
        if current_time - self.last_log_time >= self.log_interval:
            # Create comprehensive log message
            log_data = {
                'basic': {
                    'temperature': f"{results['temperature']:.2f}°C",
                    'humidity': f"{results['humidity']:.1f}%",
                    'heat_index': f"{results['heat_index']:.2f}°C",
                    'dew_point': f"{results['dew_point']:.2f}°C"
                },
                'analysis': {
                    'comfort': results['comfort_level'],
                    'air_quality': results['air_quality'],
                    'health_risk': results['health_risk']
                },
                'trends': self.trend_direction,
                'stats': {
                    'messages_processed': self.message_count,
                    'data_points': len(self.temp_history)
                }
            }
            
            # Log summary
            self.get_logger().info(
                f"📊 SENSOR ANALYSIS SUMMARY:\n"
                f"   >> Temperature: {log_data['basic']['temperature']} (Heat Index: {log_data['basic']['heat_index']})\n"
                f"   >> Humidity: {log_data['basic']['humidity']} (Dew Point: {log_data['basic']['dew_point']})\n"
                f"   >> Comfort: {log_data['analysis']['comfort']}\n"
                f"   >> Air Quality: {log_data['analysis']['air_quality']}\n"
                f"   >> Trends: T-{log_data['trends']['temperature']}, H-{log_data['trends']['humidity']}\n"
                f"   >> Processed: {log_data['stats']['messages_processed']} messages"
            )
            
            # Publish statistics
            stats_msg = String()
            stats_msg.data = json.dumps(log_data, indent=2)
            self.stats_pub.publish(stats_msg)
            
            self.last_log_time = current_time
    
    def perform_advanced_analysis(self):
        """Perform advanced periodic analysis"""
        if len(self.temp_history) < 10:
            return
        
        # Calculate statistics
        temp_data = list(self.temp_history)
        humidity_data = list(self.humidity_history)
        
        stats = {
            'temperature': {
                'mean': np.mean(temp_data),
                'std': np.std(temp_data),
                'min': np.min(temp_data),
                'max': np.max(temp_data),
                'range': np.max(temp_data) - np.min(temp_data)
            },
            'humidity': {
                'mean': np.mean(humidity_data),
                'std': np.std(humidity_data),
                'min': np.min(humidity_data),
                'max': np.max(humidity_data),
                'range': np.max(humidity_data) - np.min(humidity_data)
            }
        }
        
        # Log advanced statistics every 30 seconds
        self.get_logger().info(
            f"📈 ADVANCED ANALYTICS:\n"
            f"   Temperature: μ={stats['temperature']['mean']:.2f}°C σ={stats['temperature']['std']:.2f} "
            f"Range=[{stats['temperature']['min']:.1f}, {stats['temperature']['max']:.1f}]\n"
            f"   Humidity: μ={stats['humidity']['mean']:.1f}% σ={stats['humidity']['std']:.1f} "
            f"Range=[{stats['humidity']['min']:.0f}, {stats['humidity']['max']:.0f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedSensorCompute()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down enhanced sensor compute node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
