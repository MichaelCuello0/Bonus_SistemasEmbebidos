#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"

#define INDICADOR_LUZ_VERDE  GPIO_NUM_17
#define INDICADOR_LUZ_ROJA   GPIO_NUM_4
#define PULSADOR_AVANCE      GPIO_NUM_5
#define PULSADOR_RETROCESO   GPIO_NUM_16

#define CANAL_ANALOGICO_VELOCIDAD ADC1_CHANNEL_7

#define PIN_PUENTE_ALTO_A  GPIO_NUM_18
#define PIN_PUENTE_BAJO_A  GPIO_NUM_15
#define PIN_PUENTE_ALTO_B  GPIO_NUM_13
#define PIN_PUENTE_BAJO_B  GPIO_NUM_12

#define MODO_SENAL_PWM     LEDC_LOW_SPEED_MODE
#define TEMPORIZADOR_PWM   LEDC_TIMER_0
#define RESOLUCION_SENAL   LEDC_TIMER_8_BIT
#define FRECUENCIA_HZ_PWM  100

#define CANAL_PWM_A        LEDC_CHANNEL_0
#define CANAL_PWM_B        LEDC_CHANNEL_1
#define LIMITE_TRABAJO_PWM 255

#define PIN_TRAZO_A   GPIO_NUM_19
#define PIN_TRAZO_B   GPIO_NUM_21
#define PIN_TRAZO_C   GPIO_NUM_22
#define PIN_TRAZO_D   GPIO_NUM_23
#define PIN_TRAZO_E   GPIO_NUM_25
#define PIN_TRAZO_F   GPIO_NUM_33
#define PIN_TRAZO_G   GPIO_NUM_32

#define HABILITADOR_CEN GPIO_NUM_14
#define HABILITADOR_DEC GPIO_NUM_27
#define HABILITADOR_UNI GPIO_NUM_26


const uint8_t mapa_siete_segmentos[10][7] = {
    {0,0,0,0,0,0,1}, 
    {1,0,0,1,1,1,1}, 
    {0,0,1,0,0,1,0}, 
    {0,0,0,0,1,1,0}, 
    {1,0,0,1,1,0,0}, 
    {0,1,0,0,1,0,0}, 
    {0,1,0,0,0,0,0}, 
    {0,0,0,1,1,1,1}, 
    {0,0,0,0,0,0,0}, 
    {0,0,0,0,1,0,0}  
};

typedef enum {
    GIRO_NULO = 0,
    GIRO_DERECHA,
    GIRO_IZQUIERDA
} estado_giro_t;

gptimer_handle_t controlador_tiempo_muerto = NULL;

void inicializar_contador_microsegundos(void)
{
    gptimer_config_t ajustes_temporizador = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };
    gptimer_new_timer(&ajustes_temporizador, &controlador_tiempo_muerto);
    gptimer_enable(controlador_tiempo_muerto);
    gptimer_start(controlador_tiempo_muerto);
}

void pausa_exacta_us(uint32_t microsegundos)
{
    uint64_t tiempo_inicio, tiempo_actual;
    gptimer_get_raw_count(controlador_tiempo_muerto, &tiempo_inicio);
    do {
        gptimer_get_raw_count(controlador_tiempo_muerto, &tiempo_actual);
    } while ((tiempo_actual - tiempo_inicio) < microsegundos);
}

void establecer_entradas_salidas(void)
{
    gpio_set_direction(INDICADOR_LUZ_VERDE, GPIO_MODE_OUTPUT);
    gpio_set_direction(INDICADOR_LUZ_ROJA, GPIO_MODE_OUTPUT);

    gpio_set_direction(PULSADOR_AVANCE, GPIO_MODE_INPUT);
    gpio_set_direction(PULSADOR_RETROCESO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PULSADOR_AVANCE, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PULSADOR_RETROCESO, GPIO_PULLUP_ONLY);

    gpio_set_direction(PIN_PUENTE_ALTO_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_PUENTE_BAJO_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_PUENTE_ALTO_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_PUENTE_BAJO_B, GPIO_MODE_OUTPUT);

    gpio_set_direction(PIN_TRAZO_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_TRAZO_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_TRAZO_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_TRAZO_D, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_TRAZO_E, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_TRAZO_F, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_TRAZO_G, GPIO_MODE_OUTPUT);

    gpio_set_direction(HABILITADOR_CEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(HABILITADOR_DEC, GPIO_MODE_OUTPUT);
    gpio_set_direction(HABILITADOR_UNI, GPIO_MODE_OUTPUT);
}

void preparar_lectura_analogica(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(CANAL_ANALOGICO_VELOCIDAD, ADC_ATTEN_DB_11);
}

void ajustar_modulacion_anchos(void)
{
    ledc_timer_config_t ajustes_timer_pwm = {
        .speed_mode = MODO_SENAL_PWM,
        .duty_resolution = RESOLUCION_SENAL,
        .timer_num = TEMPORIZADOR_PWM,
        .freq_hz = FRECUENCIA_HZ_PWM,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ajustes_timer_pwm);

    ledc_channel_config_t conf_canal_a = {
        .gpio_num = PIN_PUENTE_ALTO_A,
        .speed_mode = MODO_SENAL_PWM,
        .channel = CANAL_PWM_A,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = TEMPORIZADOR_PWM,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&conf_canal_a);

    ledc_channel_config_t conf_canal_b = {
        .gpio_num = PIN_PUENTE_ALTO_B,
        .speed_mode = MODO_SENAL_PWM,
        .channel = CANAL_PWM_B,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = TEMPORIZADOR_PWM,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&conf_canal_b);
}

int obtener_promedio_velocidad(void)
{
    int acumulador_lecturas = 0;

    for (int i = 0; i < 10; i++)
    {
        acumulador_lecturas += adc1_get_raw(CANAL_ANALOGICO_VELOCIDAD);
        pausa_exacta_us(200);
    }

    return acumulador_lecturas / 10;
}

void desactivar_comunes_display(void)
{
    gpio_set_level(HABILITADOR_CEN, 1);
    gpio_set_level(HABILITADOR_DEC, 1);
    gpio_set_level(HABILITADOR_UNI, 1);
}

void apagar_trazos(void)
{
    gpio_set_level(PIN_TRAZO_A, 1);
    gpio_set_level(PIN_TRAZO_B, 1);
    gpio_set_level(PIN_TRAZO_C, 1);
    gpio_set_level(PIN_TRAZO_D, 1);
    gpio_set_level(PIN_TRAZO_E, 1);
    gpio_set_level(PIN_TRAZO_F, 1);
    gpio_set_level(PIN_TRAZO_G, 1);
}

void encender_patron_numero(int cifra)
{
    gpio_set_level(PIN_TRAZO_A, mapa_siete_segmentos[cifra][0]);
    gpio_set_level(PIN_TRAZO_B, mapa_siete_segmentos[cifra][1]);
    gpio_set_level(PIN_TRAZO_C, mapa_siete_segmentos[cifra][2]);
    gpio_set_level(PIN_TRAZO_D, mapa_siete_segmentos[cifra][3]);
    gpio_set_level(PIN_TRAZO_E, mapa_siete_segmentos[cifra][4]);
    gpio_set_level(PIN_TRAZO_F, mapa_siete_segmentos[cifra][5]);
    gpio_set_level(PIN_TRAZO_G, mapa_siete_segmentos[cifra][6]);
}

void refrescar_posicion_display(int posicion, int magnitud)
{
    desactivar_comunes_display();
    encender_patron_numero(magnitud);

    switch (posicion)
    {
        case 2: gpio_set_level(HABILITADOR_CEN, 0); break;
        case 3: gpio_set_level(HABILITADOR_DEC, 0); break;
        case 4: gpio_set_level(HABILITADOR_UNI, 0); break;
    }

    pausa_exacta_us(2000);
    desactivar_comunes_display();
}

void imprimir_valor_porcentual(int magnitud_final)
{
    if (magnitud_final < 0) magnitud_final = 0;
    if (magnitud_final > 100) magnitud_final = 100;

    int bloque_cien = magnitud_final / 100;
    int bloque_diez = (magnitud_final / 10) % 10;
    int bloque_uno  = magnitud_final % 10;

    if (magnitud_final == 100)
        refrescar_posicion_display(2, bloque_cien);
    else
    {
        desactivar_comunes_display();
        apagar_trazos();
        pausa_exacta_us(2000);
    }

    if (magnitud_final >= 10)
        refrescar_posicion_display(3, bloque_diez);
    else
    {
        desactivar_comunes_display();
        apagar_trazos();
        pausa_exacta_us(2000);
    }

    refrescar_posicion_display(4, bloque_uno);
}


static inline void activar_bajo_a(void)    { gpio_set_level(PIN_PUENTE_BAJO_A, 0); }
static inline void desactivar_bajo_a(void) { gpio_set_level(PIN_PUENTE_BAJO_A, 1); }

static inline void activar_bajo_b(void)    { gpio_set_level(PIN_PUENTE_BAJO_B, 0); }
static inline void desactivar_bajo_b(void) { gpio_set_level(PIN_PUENTE_BAJO_B, 1); }

void aplicar_ciclo_alto_a(int nivel_potencia)
{
    if (nivel_potencia < 0) nivel_potencia = 0;
    if (nivel_potencia > 100) nivel_potencia = 100;

    uint32_t ciclo_trabajo_calculado = (nivel_potencia * LIMITE_TRABAJO_PWM) / 100;
    ledc_set_duty(MODO_SENAL_PWM, CANAL_PWM_A, ciclo_trabajo_calculado);
    ledc_update_duty(MODO_SENAL_PWM, CANAL_PWM_A);
}

void aplicar_ciclo_alto_b(int nivel_potencia)
{
    if (nivel_potencia < 0) nivel_potencia = 0;
    if (nivel_potencia > 100) nivel_potencia = 100;

    uint32_t ciclo_trabajo_calculado = (nivel_potencia * LIMITE_TRABAJO_PWM) / 100;
    ledc_set_duty(MODO_SENAL_PWM, CANAL_PWM_B, ciclo_trabajo_calculado);
    ledc_update_duty(MODO_SENAL_PWM, CANAL_PWM_B);
}

void detener_pulsos_altos(void)
{
    ledc_set_duty(MODO_SENAL_PWM, CANAL_PWM_A, 0);
    ledc_update_duty(MODO_SENAL_PWM, CANAL_PWM_A);

    ledc_set_duty(MODO_SENAL_PWM, CANAL_PWM_B, 0);
    ledc_update_duty(MODO_SENAL_PWM, CANAL_PWM_B);
}

void frenar_actuador_completo(void)
{
    detener_pulsos_altos();
    desactivar_bajo_a();
    desactivar_bajo_b();
}

void rotar_sentido_reloj(int ritmo_giro)
{
    frenar_actuador_completo();
    pausa_exacta_us(200);

    desactivar_bajo_a();
    activar_bajo_b();

    aplicar_ciclo_alto_b(0);
    aplicar_ciclo_alto_a(ritmo_giro);
}

void rotar_sentido_inverso(int ritmo_giro)
{
    frenar_actuador_completo();
    pausa_exacta_us(200);

    desactivar_bajo_b();
    activar_bajo_a();

    aplicar_ciclo_alto_a(0);
    aplicar_ciclo_alto_b(ritmo_giro);
}

void app_main(void)
{
    establecer_entradas_salidas();
    preparar_lectura_analogica();
    ajustar_modulacion_anchos();
    inicializar_contador_microsegundos();

    int registro_previo_boton_avance = 1;
    int registro_previo_boton_retroceso = 1;

    gpio_set_level(INDICADOR_LUZ_VERDE, 0);
    gpio_set_level(INDICADOR_LUZ_ROJA, 0);

    desactivar_comunes_display();
    apagar_trazos();

    estado_giro_t sentido_actual_operacion = GIRO_NULO;
    int magnitud_actual = 0;

    frenar_actuador_completo();

    while (1)
    {
        int lectura_actual_avance = gpio_get_level(PULSADOR_AVANCE);
        int lectura_actual_retroceso = gpio_get_level(PULSADOR_RETROCESO);

        if (registro_previo_boton_avance == 1 && lectura_actual_avance == 0)
        {
            gpio_set_level(INDICADOR_LUZ_VERDE, 1);
            gpio_set_level(INDICADOR_LUZ_ROJA, 0);
            sentido_actual_operacion = GIRO_DERECHA;
        }

        if (registro_previo_boton_retroceso == 1 && lectura_actual_retroceso == 0)
        {
            gpio_set_level(INDICADOR_LUZ_VERDE, 0);
            gpio_set_level(INDICADOR_LUZ_ROJA, 1);
            sentido_actual_operacion = GIRO_IZQUIERDA;
        }

        registro_previo_boton_avance = lectura_actual_avance;
        registro_previo_boton_retroceso = lectura_actual_retroceso;

        int dato_crudo_sensor = obtener_promedio_velocidad();
        magnitud_actual = (dato_crudo_sensor * 100) / 4095;

        if (magnitud_actual < 15)
            magnitud_actual = 0;

        if (sentido_actual_operacion == GIRO_DERECHA)
            rotar_sentido_reloj(magnitud_actual);
        else if (sentido_actual_operacion == GIRO_IZQUIERDA)
            rotar_sentido_inverso(magnitud_actual);
        else
            frenar_actuador_completo();

        for (int i = 0; i < 25; i++)
        {
            imprimir_valor_porcentual(magnitud_actual);
        }

        printf("ADC: %d | Velocidad: %d | Dir: %d\n", dato_crudo_sensor, magnitud_actual, sentido_actual_operacion);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}