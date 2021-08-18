# LCD, teclado matricial y potenciómetro

Se uso el software STM32CubeIDE

## Configuraciones para el uso de uart2

Abrir la configuración del archivo .ioc, una vez abierto el archivo hacer lo siguiente:

- En la imagen del microcontrolador, activar TX y RX para el UART2, pines PA2 y PA3 respectivamente.
- En la pestaña "Connectivity" ir a UART2 para colocar la configuración de modo asincrono y deshabilidar RS232
- Ajustar el baudrate deseado (115200 por defecto), word length 8 bits, bit de paridad en none, bit de parada en 1


## Configuraciones para el uso de printf y scanf
Después de habilidar UART2, para poder mostrar y pedir datos se puede hacer lo siguiente:

- Copiar los archivos retarget.h y retarget.c en ./nombreProyecto/Core/Inc./nombreProyecto/Core/Src respectivamente.
- Asegurarse de indicar al compilador las operaciones con punto flotante agregando las banderas "-u _printf_float" y "-u _scanf_float"
- Inicializar la comunicación UART2 con la llamada a la función: RetargetInit(&huart2);


## Enlaces de ayuda
https://shawnhymel.com/1873/how-to-use-printf-on-stm32/


