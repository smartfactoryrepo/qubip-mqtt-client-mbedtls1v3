# MQTT Client Example with TLS 1.3 Encryption on NUCLEO-F429ZI

This example demonstrates how to establish a secure connection to a MQTT broker using TLS v1.3 encryption on a NUCLEO-F429ZI board. It then sends a MQTT message every second.

## Features 🧰

* Secure connection to mqtt broker using TLS v1.3. 🔐
* Periodic MQTT message publishing (every second). ⏱️
* Utilizes NUCLEO-F429ZI board for implementation. 💻

## Requirements ✅

* **NUCLEO-F429ZI board:** This example is specifically tailored for the NUCLEO-F429ZI development board. 
* **IDE:** STM32CubeIDE 1.16.0 📥
* **MQTT broker:** You'll need an MQTT broker that supports TLS 1.3 connections. 🧩

## How to Run 🚀

1. **Compile:**
    Compile the example code along with your chosen MQTT broker server and TLS version. 🧑‍💻

2. **Execute:**
    Flash the compiled code onto the NUCLEO-F429ZI board. 
    Run the code on the board. 🏃‍♀️

## Important Notes ⚠️

* **Network Connectivity:** Make sure you have an active internet connection to reach the server. 🌐
* **Broker Requirements:** The broker might have specific requirements for client authentication or topic subscriptions. Refer to their documentation if needed. 📖
* **Error Handling:** The example code might need to be enhanced with proper error handling and logging for production environments. 🐛

## Disclaimer 📢

This is a basic example. Adapt and extend it as per your specific use case and security requirements. 🛠️

## License 📄

This code is provided under the [MIT License](LICENSE). Feel free to use and modify it. 👍

## Contributing 🤝

Contributions are welcome! Please submit a pull request or open an issue if you find any bugs or have suggestions for improvement. 🙏
