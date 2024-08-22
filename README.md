![image](https://github.com/user-attachments/assets/e61e9b79-ca9f-47cf-afa9-fb2be1861115)# STM32-L3G4250D-and-ESP8266-
Reading the values ​​of the MEMS motion sensor 3-axis digital output gyroscope inside the STM32 developer board, sending this value over WiFi and reading the values ​​from the ROS2 node.
All pins on the STM32F072B developer board project are actively turned on


The MEMS sensor communicates via SPI as seen in PC0. You should get details about the sensor's communication from the datasheet. Before moving on to the code part, we must adjust the SPI settings of our device according to the communication of the sensor.
![image](https://github.com/user-attachments/assets/9bbfb8bd-6c3f-40ec-936e-5c3cbce82494)
We need to adjust the Data Size section.


![image](https://github.com/user-attachments/assets/40710540-5330-4d59-94b7-500c4b952003)

![image](https://github.com/user-attachments/assets/5cc74a43-3e76-4971-8198-2bb0aa17008c)

To activate the sensor's axes, we write the value 1 to the necessary registers.

WriteData(0x20,0x0F); // 0x0F = 00001111 that means  PD power mode active , Zen Yen Xen axis active.

void WriteData(uint8_t adress,uint8_t WData)
{
  uint8_t data[2] = {adress, WData};

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Set CS low
  HAL_SPI_Transmit(&hspi2, data, 2, 100); // Transmit address and value
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Set CS high
}



![espfoto](https://github.com/user-attachments/assets/4df6490f-6979-4e3d-b4cc-de0747119285)
ESP9266 mini WiFi module does not support the latest version of the esp library, so use version 2.5.5. Additionally, since there is no grounding in the module, I used the following steps: I plugged the G0 and GND pins of the UART-USB converter into the breadboard. As seen in the figure, green and yellow cable is GND and red and orange cable is G0. In order to put the code into the ESP8266, we must plug it into the computer with G0 connected to GND and then disconnect G0 from GND. The red cable in between allows me to do this manually. There are people in YouTube videos who do this by soldering a button between G0 and GND.







