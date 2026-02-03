

# MQTT Client Example with TLS 1.3 Encryption on NUCLEO-F429ZI with temperature sensor

This example demonstrates how to establish a secure connection to a MQTT broker using TLS v1.3 encryption on a NUCLEO-F429ZI board. It subsequently establishes a Modbus TCP connection to a PLC, reads a holding register, and transmits its value to the MQTT broker. The register is then incremented by one, updated on the PLC, and this cycle repeats continuously. In this example, a DS18B20 one-wire temperature sensor is used. The temperature is read and sent to the MQTT broker on another topic.

## Features 🧰

- Secure connection to MQTT broker using TLS v1.3. 🔐
- Periodic MQTT message publishing (every second). ⏱️
- Periodic Modbus TCP communication. 🔄
- Utilizes NUCLEO-F429ZI board for implementation. 💻

## Requirements ✅

- **NUCLEO-F429ZI board:** This example is specifically tailored for the NUCLEO-F429ZI development board.
- **IDE:** STM32CubeIDE 1.16.0 📥
- **MQTT broker:** You'll need an MQTT broker that supports TLS 1.3 connections. 🧩
- **PLC:** You'll need a PLC that supports Modbus TCP.
- **Temperature Sensor:** A DS18B20 one-wire temperature sensor is required.

## How to Connect the Temperature Sensor 🛠️

1. **Connect Data Pin:** Connect the data pin of the DS18B20 sensor to PC12 on the NUCLEO-F429ZI board

2. **Pull-Up Resistor:**
   Use a 4.7kΩ pull-up resistor between the data pin and the VCC pin to ensure proper communication.

3. **Power and Ground:** Connect the VCC pin of the sensor to a 3.3V or 5V (PC12 is 5 V tolerant) power supply pin on the board, and the GND pin to a ground pin on the board.

4. **Verify Connections:**
   Double-check all connections to ensure proper wiring and avoid damage to the sensor or board.

## How to Run 🚀

1. **Compile:**
   Compile the example code along with your chosen MQTT broker server and TLS version. 🧑‍💻

2. **Execute:**
   Flash the compiled code onto the NUCLEO-F429ZI board.
   Run the code on the board. 🏃‍♀️

## Important Notes ⚠️

- **Network Connectivity:** Make sure you have an active internet connection to reach the server. 🌐
- **Broker Requirements:** The broker might have specific requirements for client authentication or topic subscriptions. Refer to their documentation if needed. 📖
- **Error Handling:** The example code might need to be enhanced with proper error handling and logging for production environments. 🐛

## Disclaimer 📢

This is a basic example. Adapt and extend it as per your specific use case and security requirements. 🛠️

## License 📄

This code is provided under the [MIT License](LICENSE). Feel free to use and modify it. 👍

## Contributing 🤝

Contributions are welcome! Please submit a pull request or open an issue if you find any bugs or have suggestions for improvement. 🙏

## Todo 🚧

1. Add post-quantum secure element integration. 🛡️

