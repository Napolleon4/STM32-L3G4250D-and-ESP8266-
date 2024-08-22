![image](https://github.com/user-attachments/assets/e61e9b79-ca9f-47cf-afa9-fb2be1861115)# STM32-L3G4250D-and-ESP8266-
Reading the values ​​of the MEMS motion sensor:&lt;br> 3-axis digital output gyroscope inside the STM32 developer board, sending this value over WiFi and reading the values ​​from the ROS2 node.
All pins on the STM32F072B developer board project are actively turned on
![image](https://github.com/user-attachments/assets/84a70f96-9dfb-4e8a-94ec-a37fdeed6853)

The MEMS sensor communicates via SPI as seen in PC0. You should get details about the sensor's communication from the datasheet. Before moving on to the code part, we must adjust the SPI settings of our device according to the communication of the sensor.
![image](https://github.com/user-attachments/assets/9bbfb8bd-6c3f-40ec-936e-5c3cbce82494)
We need to adjust the Data Size section.



![image](https://github.com/user-attachments/assets/5cc74a43-3e76-4971-8198-2bb0aa17008c)
To activate the axes of the sensor, we activate it by writing the value 1 to the necessary registers.
![image](https://github.com/user-attachments/assets/40710540-5330-4d59-94b7-500c4b952003)

void WriteData(uint8_t adress,uint8_t WData)
{
  uint8_t data[2] = {adress, WData};

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Set CS low
  HAL_SPI_Transmit(&hspi2, data, 2, 100); // Transmit address and value
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Set CS high
}


