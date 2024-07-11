#include <reg51.h>
#include <stdio.h>
#include <math.h>

// Definir pines para el LCD
sbit RS = P3 ^ 0;
sbit EN = P3 ^ 1;
sbit D4 = P3 ^ 2;
sbit D5 = P3 ^ 3;
sbit D6 = P3 ^ 4;
sbit D7 = P3 ^ 5;

// Bit boton
sbit BOTON = P2 ^ 0;

// Definir pines LED
sbit LED1 = P1 ^ 1;
sbit LED2 = P1 ^ 2;
sbit LED3 = P1 ^ 3;
sbit LED4 = P1 ^ 4;

// Definir pines ECHO
sbit TRIG_PIN = P1 ^ 6; // Pin de disparo (TRIG)
sbit ECHO_PIN = P2 ^ 2; // Pin de eco (ECHO)

// Definiciones para pines I²C
sbit SDA = P2 ^ 3; // Pin de datos I²C
sbit SCL = P2 ^ 4; // Pin de reloj I²C

// Definir puertos motor
sbit MOTOR1_FWD = P1 ^ 5;
sbit MOTOR1_BWD = P1 ^ 7;
sbit MOTOR2_FWD = P3 ^ 6;
sbit MOTOR2_BWD = P3 ^ 7;

// Direcciones del ADS1115
#define ADS1115_ADDRESS 0x48 // Dirección I²C del ADS1115

// Registro de configuración del ADS1115
#define CONFIG_REGISTER 0x01
#define CONVERSION_REGISTER 0x00

// Prototipos de funciones
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(unsigned char);
unsigned char I2C_Read(unsigned char);
void ADS1115_WriteConfig(unsigned int);
unsigned int ADS1115_ReadConversion(void);
void controlar_leds(unsigned int adc_value);

// Función de retardo
void delay(unsigned int);

// Variables flanco ascendente
unsigned int estado_anterior;

// Variable medida distancia
unsigned int distance;
unsigned int adc_value;

// Prototipos de funciones del LCD
void LCD_init();
void LCD_cmd(unsigned char cmd);
void LCD_data(unsigned char datos);
void LCD_write_string(const char *str);

// Prototipos secuencia leds
void encender_leds_secuencialmente();

// Definicion funciones
void trigger_pulse();
void delay_us(unsigned int us);
unsigned int measure_distance();
void int_to_string(unsigned int num, char *str);

// Función para inicializar el temporizador 0 en modo 1
void timer0_init()
{
    TMOD &= 0xF0; // Limpiar bits de Timer 0
    TMOD |= 0x01; // Configurar Timer 0 en modo 1 (16 bits)
}

// Función para generar un retardo de 1 microsegundo usando Timer 0
void delay_1us()
{
    TH0 = 0xFF; // Cargar el valor alto para 1 us (ajustado)
    TL0 = 0xF4; // Cargar el valor bajo para 1 us (ajustado)
    TR0 = 1;    // Iniciar el temporizador
    while (!TF0)
        ;    // Esperar hasta que se desborde el temporizador
    TR0 = 0; // Detener el temporizador
    TF0 = 0; // Borrar el flag de desbordamiento del temporizador
}

// Función para generar un retardo de n microsegundos
void delay_us(unsigned int us)
{
    unsigned int i;
    for (i = 0; i < us; i++)
    {
        delay_1us();
    }
}

// Función para generar un pulso de disparo al sensor ultrasónico
void trigger_pulse()
{
    TRIG_PIN = 1; // Establecer el pin de disparo en alto
    delay_us(10); // Esperar un corto periodo de tiempo (al menos 10us)
    TRIG_PIN = 0; // Apagar el pin de disparo
}

// Función para medir la distancia utilizando el sensor ultrasónico
unsigned int measure_distance()
{
    float distance_measurement, valor;
    unsigned long duration;
    unsigned int distance_cm;

    trigger_pulse();

    while (!ECHO_PIN)
        ;    // Esperar hasta que el pin de eco se active (se ponga en alto)
    TR0 = 1; // Iniciar el temporizador

    while (ECHO_PIN && !TF0)
        ;    // Esperar hasta que el pin de eco se desactive (se ponga en bajo)
    TR0 = 0; // Detener el temporizador

    // Calcular la distancia en centímetros
    valor = 1.085e-6 * 34300;
    duration = (TL0 | (TH0 << 8));        // Leer el valor del temporizador
    distance_measurement = duration / 58; // Calcular la distancia (ida y vuelta)
    distance_cm = (unsigned int)distance_measurement;

    return distance_cm;
}

// Convertir entero a string
void int_to_string(unsigned int num, char *str)
{
    int i = 0;
    int temp_num = num;

    // Contar el número de dígitos
    do
    {
        temp_num /= 10;
        i++;
    } while (temp_num != 0);

    str[i] = '\0'; // Añadir el carácter nulo al final

    // Convertir cada dígito a carácter
    while (num != 0)
    {
        str[--i] = (num % 10) + '0';
        num /= 10;
    }

    if (i == 1)
    {
        str[0] = '0';
    }
}

void delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 500; j++)
            ;
}
void motores_adelante()
{
    MOTOR1_FWD = 1;
    MOTOR1_BWD = 0;
    MOTOR2_FWD = 1;
    MOTOR2_BWD = 0;
}

void motores_parado()
{
    MOTOR1_FWD = 0;
    MOTOR1_BWD = 0;
    MOTOR2_FWD = 0;
    MOTOR2_BWD = 0;
}

void motor1_atras()
{
    MOTOR1_FWD = 0;
    MOTOR1_BWD = 1;
    MOTOR2_FWD = 1;
    MOTOR2_BWD = 0;
}

void main()
{
    char distance_str[10]; // Cadena para almacenar la distancia como texto
    char value_str[10];
    unsigned int previous_distance = 0xFFFF;  // Valor inicial que nunca será igual a una distancia medida
    unsigned int previous_adc_value = 0xFFFF; // Valor inicial que nunca será igual a un valor ADC medido

    P1 = 0x00;
    P3 = 0x00;
    P2 = 0xFF; // Configurar P2 como entrada
    estado_anterior = 0;

    timer0_init(); // Inicializar el temporizador

    I2C_Init();

    // Configurar el ADS1115 (modo continuo, AIN0, PGA ±4.096V, 128SPS)
    LCD_init(); // Inicializar el LCD
    LCD_cmd(0x80);
    LCD_write_string("Hola soy Robo ");
    encender_leds_secuencialmente(); // Encender LEDs uno tras otro con un retardo de 500ms
    LCD_cmd(0x01);                   // Limpiar la pantalla antes de escribir
    delay_ms(500);
    lcd_init();

    while (1)
    {
        // Medir la distancia utilizando el sensor ultrasónico
        ADS1115_WriteConfig(0x8583);
        distance = measure_distance();
        int_to_string(distance, distance_str); // Convertir la distancia a cadena
        adc_value = ADS1115_ReadConversion();
        int_to_string(adc_value, value_str);

        if (BOTON == 1)
        { // Si el interruptor está en posición encendido

            // Limpiar y actualizar solo la distancia si ha cambiado

            LCD_cmd(0x80);                   // Ir al inicio de la primera línea
            LCD_write_string("D:         "); // Espacios para limpiar la línea
            LCD_cmd(0x80);                   // Ir al inicio de la primera línea
            LCD_write_string("D: ");         // Mostrar "Distancia: " en el LCD
            LCD_write_string(distance_str);  // Mostrar la distancia medida en el LCD
            LCD_write_string(" cm");         // Mostrar " cm" en el LCD
                                             // Limpiar y actualizar solo el valor ADC si ha cambiado
            LCD_cmd(0xC0);                   // Mover a la segunda línea del LCD
            LCD_write_string("AD:        "); // Espacios para limpiar la línea
            LCD_cmd(0xC0);                   // Mover a la segunda línea del LCD
            LCD_write_string("AD: ");        // Mostrar "ADC: " en el LCD
            LCD_write_string(value_str);

            previous_adc_value = adc_value;
            previous_distance = distance;
            delay_ms(50);
            if (distance > 15)
            {
                motores_adelante();
            }
            else
            {
                motores_parado();
                delay_ms(50);
                motor1_atras();
                delay_ms(50);
            }

            controlar_leds(adc_value);
            estado_anterior = 1;
        }
        else
        {

            lcd_init();
            LCD_write_string("AD: "); // Mostrar "ADC: " en el LCD
            LCD_write_string(value_str);
            controlar_leds(adc_value);
            delay_ms(50);
            motores_parado();

            estado_anterior = 0;
        }

        // Esperar un periodo de tiempo antes de realizar la próxima medición

        delay_ms(10);
    }
}

// Inicialización del LCD
void LCD_init()
{
    delay_ms(10);  // Espera de inicialización
    LCD_cmd(0x02); // Modo de 4 bits
    LCD_cmd(0x28); // 2 líneas, 5x8 matriz de puntos
    LCD_cmd(0x0E); // Mostrar cursor
    LCD_cmd(0x06); // Incrementar el cursor
    LCD_cmd(0x01); // Limpiar pantalla
    delay_ms(10);  // Espera de limpieza de pantalla
}

// Enviar comando al LCD
void LCD_cmd(unsigned char cmd)
{
    RS = 0; // Modo comando
    D4 = (cmd >> 4) & 0x01;
    D5 = (cmd >> 4) & 0x02;
    D6 = (cmd >> 4) & 0x04;
    D7 = (cmd >> 4) & 0x08;
    EN = 1;
    delay_us(1);
    EN = 0;
    delay_us(1);
    D4 = cmd & 0x01;
    D5 = cmd & 0x02;
    D6 = cmd & 0x04;
    D7 = cmd & 0x08;
    EN = 1;
    delay_us(1);
    EN = 0;
    delay_ms(2); // Espera para comando
}

// Enviar datos al LCD
void LCD_data(unsigned char datos)
{
    RS = 1; // Modo datos
    D4 = (datos >> 4) & 0x01;
    D5 = (datos >> 4) & 0x02;
    D6 = (datos >> 4) & 0x04;
    D7 = (datos >> 4) & 0x08;
    EN = 1;
    delay_us(1);
    EN = 0;
    delay_us(1);
    D4 = datos & 0x01;
    D5 = datos & 0x02;
    D6 = datos & 0x04;
    D7 = datos & 0x08;
    EN = 1;
    delay_us(1);
    EN = 0;
    delay_ms(2); // Espera para datos
}

// Función para escribir una cadena en el LCD
void LCD_write_string(const char *str)
{
    while (*str)
    {
        LCD_data(*str++);
    }
}
void I2C_Init(void)
{
    SDA = 1;
    SCL = 1;
}

void I2C_Start(void)
{
    SDA = 1;
    SCL = 1;
    SDA = 0;
    SCL = 0;
}

void I2C_Stop(void)
{
    SCL = 0;
    SDA = 0;
    SCL = 1;
    SDA = 1;
}

void I2C_Write(unsigned char dat)
{
    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        SDA = (dat & 0x80) >> 7;
        SCL = 1;
        dat <<= 1;
        SCL = 0;
    }
    SDA = 1; // Liberar línea de datos para ACK
    SCL = 1;
    SCL = 0;
}

unsigned char I2C_Read(unsigned char ack)
{
    unsigned char i, dat = 0;
    SDA = 1; // Liberar línea de datos para entrada
    for (i = 0; i < 8; i++)
    {
        dat <<= 1;
        SCL = 1;
        dat |= SDA;
        SCL = 0;
    }
    SDA = ack ? 0 : 1; // ACK o NACK
    SCL = 1;
    SCL = 0;
    return dat;
}

void ADS1115_WriteConfig(unsigned int config)
{
    I2C_Start();
    I2C_Write((ADS1115_ADDRESS << 1) | 0); // Dirección + bit de escritura
    I2C_Write(CONFIG_REGISTER);            // Registro de configuración
    I2C_Write((config >> 8) & 0xFF);       // Byte alto de la configuración
    I2C_Write(config & 0xFF);              // Byte bajo de la configuración
    I2C_Stop();
}

unsigned int ADS1115_ReadConversion(void)
{
    unsigned int value;
    I2C_Start();
    I2C_Write((ADS1115_ADDRESS << 1) | 0); // Dirección + bit de escritura
    I2C_Write(CONVERSION_REGISTER);        // Registro de conversión
    I2C_Start();
    I2C_Write((ADS1115_ADDRESS << 1) | 1);    // Dirección + bit de lectura
    value = ((unsigned int)I2C_Read(1) << 8); // Leer byte alto
    value |= I2C_Read(0);                     // Leer byte bajo
    I2C_Stop();
    return value;
    adc_value = value;
}

void delay(unsigned int count)
{
    unsigned int i, j;
    for (i = 0; i < count; i++)
        for (j = 0; j < 1275; j++)
            ;
}
// Función para encender LEDs uno tras otro con un retardo de 500ms
void encender_leds_secuencialmente()
{

    LED1 = 1;     // Encender LED1
    delay_ms(50); // Esperar 500ms
    LED1 = 0;     // Apagar LED1
    LED2 = 1;     // Encender LED2
    delay_ms(50); // Esperar 500ms
    LED2 = 0;     // Apagar LED2
    LED3 = 1;     // Encender LED3
    delay_ms(50); // Esperar 500ms
    LED3 = 0;     // Apagar LED3
    LED4 = 1;     // Encender LED4
    delay_ms(50); // Esperar 500ms
    LED4 = 0;     // Apagar LED4
    delay_ms(50); // Esperar 500ms
    LED4 = 1;     // Encender LED4
    delay_ms(50); // Esperar 500ms
    LED4 = 0;     // Apagar LED4
    LED3 = 1;     // Encender LED3
    delay_ms(50); // Esperar 500ms
    LED3 = 0;     // Apagar LED3
    LED2 = 1;     // Encender LED2
    delay_ms(50); // Esperar 500ms
    LED2 = 0;     // Apagar LED2
    LED1 = 1;     // Encender LED1
    delay_ms(50); // Esperar 500ms
    LED1 = 0;     // Apagar LED1
}
void controlar_leds(unsigned int adc_value)
{
    // Apagar todos los LEDs inicialmente
    LED1 = 0;
    LED2 = 0;
    LED3 = 0;
    LED4 = 0;

    // Encender los LEDs según el valor del ADC
    if (adc_value < 1024)
    {
        LED1 = 1;
        LED2 = 1;
        LED3 = 1;
        LED4 = 1;
    }
    else if (adc_value < 2048)
    {
        LED1 = 0;
        LED2 = 1;
        LED3 = 1;
        LED4 = 0;
    }
    else if (adc_value < 3072)
    {
        LED1 = 1;
        LED2 = 0;
        LED3 = 0;
        LED4 = 1;
    }
    else
    {
        LED1 = 0;
        LED2 = 0;
        LED3 = 0;
        LED4 = 0;
    }
}