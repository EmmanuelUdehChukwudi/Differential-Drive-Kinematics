from flask import Flask, request, jsonify, render_template, redirect, url_for

app = Flask(__name__)

# Global variables to store the wheel speeds, set point, and update rate
wheel_speeds = {"left_speed": 0, "right_speed": 0}
set_point = 0.0  # Default set point
update_rate = 1000  # Default update rate in milliseconds

@app.route('/')
def home():
    global wheel_speeds, set_point, update_rate
    return render_template('index.html', wheel_speeds=wheel_speeds, set_point=set_point, update_rate=update_rate)

@app.route('/update_speeds', methods=['POST'])
def update_speeds():
    global wheel_speeds
    data = request.get_json()
    if data and "wheel_speeds" in data:
        try:
            # Parse the wheel speeds from the received data
            speeds = data["wheel_speeds"].split(',')
            if len(speeds) != 2:
                raise ValueError("Expected two values for wheel speeds")
            left_speed = int(speeds[0].strip())
            right_speed = int(speeds[1].strip())
            
            # Update the global wheel speeds
            wheel_speeds["left_speed"] = left_speed
            wheel_speeds["right_speed"] = right_speed

            print(f"Updated wheel speeds: Left - {left_speed}, Right - {right_speed}")
            return jsonify({"message": "Wheel speeds updated successfully"}), 200
        except (IndexError, ValueError) as e:
            print(f"Error parsing wheel speeds: {e}")
            return jsonify({"message": "Invalid data format"}), 400
    else:
        return jsonify({"message": "Invalid data received"}), 400

@app.route('/get_wheel_speeds', methods=['GET'])
def get_wheel_speeds():
    global wheel_speeds
    return jsonify(wheel_speeds), 200

@app.route('/get_setpoint', methods=['GET'])
def get_setpoint():
    global set_point
    return jsonify({"set_point": set_point}), 200

@app.route('/set_setpoint', methods=['POST'])
def set_setpoint():
    global set_point
    data = request.get_json()
    if data and "set_point" in data:
        set_point = float(data["set_point"])
        return jsonify({"message": "Set point updated successfully"}), 200
    else:
        return jsonify({"message": "Invalid data received"}), 400

@app.route('/update_setpoint', methods=['POST'])
def update_setpoint():
    global set_point
    set_point = float(request.form['set_point'])
    return redirect(url_for('home'))

@app.route('/get_update_rate', methods=['GET'])
def get_update_rate():
    global update_rate
    return jsonify({"update_rate": update_rate}), 200

@app.route('/set_update_rate', methods=['POST'])
def set_update_rate():
    global update_rate
    data = request.get_json()
    if data and "update_rate" in data:
        update_rate = int(data["update_rate"])
        return jsonify({"message": "Update rate updated successfully"}), 200
    else:
        return jsonify({"message": "Invalid data received"}), 400

if __name__ == '__main__':
    app.run(host='192.168.43.81', port=5000, debug=True)
