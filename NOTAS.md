
## Se cambiaron los HALDelay por delays manuales con while porque la interrupcion no los acepta cuando se usa en el teclado

## Teclado funcionando en pines PA8-PA15, configuracion PA8-PA11(Renglones) como OUTPUT PUSH-PULL, NO-PULL, PA12-PA15(Columnas) INPUT, PULL-UP

## Pantalla funcionando en pines PA0, PA1, PA4, PA6, PA7

## PA5 es usado para mostrar la interrupcion activa en el led de la placa

## PC13 el pin relacionado al boton y el que activa la interrupcion, configurada con resistencia pull-up y detección de flanco de bajada

## PC0 simula lectura del voltaje en el termistor

## Símbolo de grados ° agregado, se usa colocando el caracter ~ (Virgulilla)

## La funcion Hal\_Delay() no funciona en la funcion de interrupción por lo que los HAL\_Delay en el archivo keypad.c se remplazaron por while's simples

