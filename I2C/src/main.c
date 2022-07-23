//////////////////////////////////////////////////////////////////////////////
//                            HION TECNOLOGIA                               //
//                          PROJETO BMS - KINDER                            //
//                                                                          //
//                         Software Teste SLEEP ESP32                       //
//////////////////////////////////////////////////////////////////////////////
//    Modulo:               DEV KIT ESP                                     //
// Descrição:                                                               //
//   Este software sera a base para o teste da fubnção sleep awake da esp   //
//   Autor:                    Jesse                                        //
//                                                                          //
// Compilador/Assembler:        Visual studio Code Version: 1.44            //
//                              Node.js: 10.16.3                            //
//                              PlatformIO Home:3.1.1 Core 4.3.1            //
//                              Espressif_IDF-ESP32 (4.0.0)                 //
//                                                                          //
// Hardware:            ESP32 DEvKITC                                       //
//                                                                          //
// Data: <19/07/2022>                                                       //
// Historico:                    Iniciais               Motivo da Mudança   //
//                            do Projetista                                 //
//                                                                          //
// 15/07/2022                        JAL               stable  (ESP-IDF 4.0)//
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//     1.      ARQUIVOS DE DEFINIÇÕES (INCLUDES)                            *
//     1.1       Arquivos includes padrões do  Compilador                   *
/////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/////////////////////////////////////////////////////////////////////////////
//      1.2   Arquivos includes do ESP-IDF - RTOS                          //
/////////////////////////////////////////////////////////////////////////////

/* bibliotecas */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/can.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/timer.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include <unistd.h>
#include "driver/i2c.h"

/////////////////////////////////////////////////////////////////////////////
//      1.3   Arquivos includes de usuário                                 //
/////////////////////////////////////////////////////////////////////////////

    //inserir Aqui

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//      2.      DECLARAÇÕES                                                //
//      2.1       Constantes Internas                                      //
/////////////////////////////////////////////////////////////////////////////

#define I2C_MASTER_FREQ_HZ      400000 // Frequencia do clock master

//INA219
#define CONFIG_219              0x00 // Registrador de configuracao
#define TENSAO_RSHUNT_219       0x01 // Registrador com a Tensao do Shunt 
#define TENSAO_BUS_219          0x02 // Registrador com a Tensao do BUS
#define POTENCIA_W_219          0x03 // Registrador com a Potencia [Watts] 
#define CORRENTE_A_219          0x04 // Registrador com a Corrente [mA]
#define CALIBRAC_219            0x05 // Registrador de Calibracao

// Define escrita e leitura do endereco do slave
#define ADDRESS_WRITE           0x80 // Endereco Escravo INA - Escrita
#define ADDRESS_READ            0x81 // Endereco Escravo INA - Leitura

//GPIO
#define GPIO_SDA                21 // Pino 21 definido como SDA
#define GPIO_SCL                22 // Pino 22 definido como SCL

//LED

#define DESLIGAR    0                   //COMANDO DESLIGAR  / DESATIVAR
#define LIGAR       1                   //COMANDO LIGAR / ACIONAR
#define TOGGLE      2                   //COMANDO PARA ALTERAR O ESTADO ATUAL
#define BUF_SIZE    500                 //Alocacao do buffer do uart na RAM

/////////////////////////////////////////////////////////////////////////////
//      2.2   Definições de entradas e saídas                               //
/////////////////////////////////////////////////////////////////////////////
#define TXD_UART1  (GPIO_NUM_1)         //PINO TX UART1 = GPIO1
#define RXD_UART1  (GPIO_NUM_3)         //PINO RX UART1 = GPIO3
#define RTS_UART  (UART_PIN_NO_CHANGE)
#define CTS_UART  (UART_PIN_NO_CHANGE)

#define LED_ESP32 (GPIO_NUM_12)       //PINO LATCH (RCLK) DO ESPANSOR DE OUTPUTS
#define INP17 (GPIO_NUM_17)
#define INP10 (GPIO_NUM_10)
#define INP15 (GPIO_NUM_15)

/////////////////////////////////////////////////////////////////////////////
//      3.   GLOBAIS  TYPEDEF STRUCTs                                      //
/////////////////////////////////////////////////////////////////////////////

//inserir aqui

/////////////////////////////////////////////////////////////////////////////
//      3.1   Variáveis Globais  (Valores Default da Aplicação)
/////////////////////////////////////////////////////////////////////////////

//inserir aqui

/////////////////////////////////////////////////////////////////////////////
//      3.1   Variáveis Globais  (Valores Default da Aplicação)
/////////////////////////////////////////////////////////////////////////////
//inserir aqui

/////////////////////////////////////////////////////////////////////////////
// RTOS Task Handles
/////////////////////////////////////////////////////////////////////////////
  //inserir Aqui

/////////////////////////////////////////////////////////////////////////////
// RTOS Queue Handles
/////////////////////////////////////////////////////////////////////////////
  //inserir Aqui

/////////////////////////////////////////////////////////////////////////////
// RTOS Semaphore Handles
/////////////////////////////////////////////////////////////////////////////
  //inserir Aqui

/////////////////////////////////////////////////////////////////////////////
//      3.2.   INTERRUPÇÕES                                                //
/////////////////////////////////////////////////////////////////////////////
  //inserir Aqui

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
//      4.   PROTOTIPOS TAREFAS E FUNCOES                                  //
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//                   Nome da função: uart_task(void *arg)
//
// Proposito: Realiza monitoramento e a atuação na UART 1 e 2
//
//   Entrada: void *arg
//     Saída: NULL
/////////////////////////////////////////////////////////////////////////////
void uart_task(void *arg) {
    while (1) {
        // Configure a temporary buffer for the incoming data
        uint8_t input17 = gpio_get_level(INP17);
        if (!input17) {
            uart_write_bytes(UART_NUM_1, "FUNCIONA \r\n", 11);
            
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
} //Final da TASK uart_task
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//                   Nome da função: xLED(void *arg)
//
// Proposito: Piscar LED para teste da ESP32
//
//   Entrada: void *arg
//     Saída: NULL
/////////////////////////////////////////////////////////////////////////////
void xLED(void *arg) {

    uint8_t led = DESLIGAR;

    while(1) {
        vTaskDelay(500 / portTICK_PERIOD_MS);

        led = !led;
        gpio_set_level(LED_ESP32, led);
    }
} //Final da TASK uart_task
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//      5.      FUNÇÃO PRINCIPAL                                           //
/////////////////////////////////////////////////////////////////////////////
// Proposito: Configura os dados da ESP
//==============================================================================
void Configura_Master(void) {

    // Configuracao do Master
    int i2c_master_port = 0;
        i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,        // Indica que vai configurar o master
        .sda_io_num = GPIO_SDA,         // Indica o GPIO que vai ser usado na linha SDA (21)
        .sda_pullup_en = GPIO_PULLUP_ENABLE,        // Utiliza o resistor de pull up interno
        .scl_io_num = GPIO_SCL,         // Indica o GPIO que vai ser usado na linha SCL (22)
        .scl_pullup_en = GPIO_PULLUP_ENABLE,        // Utiliza o resistor de pull up interno
        .master.clk_speed = I2C_MASTER_FREQ_HZ,     // Utiliza a frequencia definida (400 kHz)
        };

        i2c_param_config(I2C_NUM_0, &conf);                             // Configura parametros da porta
        i2c_driver_install ( I2C_NUM_0, I2C_MODE_MASTER, 0 , 0 , 0);    // Instala o driver
}

//==============================================================================
//          Nome da funcao: void Configura_INA(void) {
//
// Proposito: Configuracao dos dados recebidos.
//==============================================================================
void Configura_INA(void) {
    i2c_cmd_handle_t cmd = NULL;

    // (0x19, 0x9F) bus 16V, shunt 320mV, 12-bit continuous Vshunt Vbus
    uint8_t dados [3] = {CONFIG_219, 0x19, 0x9F};

        // Manda as informacoes de configuracao  
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);                     // Inicializa comunicacao
        i2c_master_write_byte(cmd, ADDRESS_WRITE, false);       // Manda o endereco de escrita
        i2c_master_write(cmd, dados, 3, false);                 // Manda os dados de configuracao
        i2c_master_stop(cmd);                      // Para comunicacao
        i2c_master_cmd_begin ( I2C_NUM_0, cmd ,(1/portTICK_RATE_MS));  // Executa o comando acima 
        i2c_cmd_link_delete ( cmd );               // Para o comando
}

//==============================================================================
//          Nome da funcao: void Calibra_INA(void) {
//
// Proposito: Configuracao a calibracao dos dados recebidos pelo INA.
//==============================================================================
void Calibra_INA(void) {
    i2c_cmd_handle_t cmd = NULL;
    
    // (0x10, 0x00) 1m A/bit, 20m W/bit
    uint8_t dados [3] = {CALIBRAC_219, 0x10, 0x00};

        // Manda as informacoes da calibracao  
        cmd = i2c_cmd_link_create();                // Criacao do link de comunicacao
        i2c_master_start(cmd);                      // Inicializa comunicacao
        i2c_master_write_byte(cmd, ADDRESS_WRITE, false);   // Manda o endereco de escrita
        i2c_master_write_byte(cmd, TENSAO_BUS_219, false);     // Configura e chama a funcao de leitura da linha bus          
        i2c_master_stop(cmd);                       // Para comunicacao
        i2c_master_cmd_begin ( I2C_NUM_0, cmd ,(1/portTICK_RATE_MS));  // Executa o comando acima
        i2c_cmd_link_delete ( cmd );                // Para o comando                 // Para o comando  
}

//==============================================================================
//          Nome da task: void EnviaDadosI2C(void *parametos) {
//
// Proposito: Executa comunicacao 
//==============================================================================
void EnviaDadosI2C(void *parametos) {

    i2c_cmd_handle_t cmd = NULL;
    // Dados de configuracao da funcao de leitura de tensao de bus
    //uint8_t configura_bus [3] = {CORRENTE_A_219, 0x00, 0x00};
    // Guarda os bytes de resposta do slave
    uint8_t resposta[8];
    // Recebe os valores de "resposta" e realiza a conversao
    uint16_t TensaoBus = 0;
    // Mostra o valor recebido
    float VB = 0;

    while(1) {
        cmd = i2c_cmd_link_create();                // Criacao do link de comunicacao
        i2c_master_start(cmd);                      // Inicializa comunicacao
        i2c_master_write_byte(cmd, ADDRESS_WRITE, false);   // Manda o endereco de escrita
        i2c_master_write_byte(cmd, TENSAO_BUS_219, false);     // Configura e chama a funcao de leitura da linha bus
        i2c_master_start(cmd);                      // Inicializa comunicacao
        i2c_master_write_byte(cmd, ADDRESS_READ, false);    // Manda o endereco de leitura
        i2c_master_read(cmd, resposta, 2, I2C_MASTER_LAST_NACK);    // Recebe e guarda a resposta do slave
        i2c_master_stop (cmd);                       // Para comunicacao
        i2c_master_cmd_begin ( I2C_NUM_0, cmd ,(1/portTICK_RATE_MS) );  // Executa o comando acima
        i2c_cmd_link_delete (cmd);                // Para o comando
        TensaoBus = (uint16_t) ((resposta[0] << 8) | resposta[1]);     // Desloca os bits para ler a word
        TensaoBus = TensaoBus >> 3;                         // Desloca 3 para fator de leitura
        VB = TensaoBus * 0.004;        
        printf("Tensao Bus: %.2f\n\n",VB);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void app_main() {

    Configura_Master();
    // Configura_INA();
    Calibra_INA();
    xTaskCreatePinnedToCore(EnviaDadosI2C,"EnviaDadosI2C", 4096, NULL, 1, NULL, 1);
}
/////////////////////////////////////////////////////////////////////////////
//      Criação da QUEUE's do RTOS
///////////////////////////////////////////////////////////////////////////// 
    //AFE_queue = xQueueCreate(10, sizeof( struct AFE_Resp ));   //Cria Queue para Mensagem Resposta AFE

/////////////////////////////////////////////////////////////////////////////
//      Criação da Semáforos do RTOS
/////////////////////////////////////////////////////////////////////////////
    //semaforoAFE = xSemaphoreCreateBinary();

/////////////////////////////////////////////////////////////////////////////
//  Configurações iniciais do microcontrolador e criação das TASKs dp RTOS
/////////////////////////////////////////////////////////////////////////////
    //Tarefas do MCU 0 - Primeiro CORE de processamento
    //xTaskCreatePinnedToCore(uart_task, "uart_task", BUF_SIZE * 2, NULL, 10, NULL,0);//Tarefa 1 com prioridade (10) no core 0
    // xTaskCreatePinnedToCore(xLED, "xLED", configMINIMAL_STACK_SIZE, NULL, 9, NULL,0);//Tarefa 2 com prioridade (9) no core 0

    //Tarefas do MCU 1 - Segundo CORE de processamento
    //xTaskCreatePinnedToCore(coulombCounter_task, "coulombCounter_task", 4096, NULL, 10, &xHandle_COULOMB,1);//Tarefa -1 com prioridade (10) no core 1

/////////////////////////////////////////////////////////////////////////////
//      Suspensão de Tarefas que não serão utilizadas no inicio da aplicação
///////////////////////////////////////////////////////////////////////////// 
   //inserir aqui

/////////////////////////////////////////////////////////////////////////////
//      Habilita as interrupções do MCU
///////////////////////////////////////////////////////////////////////////// 
   //inserir aqui

/////////////////////////////////////////////////////////////////////////////
//      Inicialização das variáveis da MAIN
/////////////////////////////////////////////////////////////////////////////
   //inserir aqui

/////////////////////////////////////////////////////////////////////////////
//     Loop principal
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//                             FIM DO ARQUIVO                              //
/////////////////////////////////////////////////////////////////////////////