/***********************************************************************************************
    Programa para controle de sumo 500g "Sombra"
    Propriedade da equipe FEG - Robótica
    Última atualizacao: 23/06/2017 - Por Guilherme "Bob" Renoldi

/***********************************************************************************************/

#include <18F2431.h>

#device adc=10

#fuses HS, NOWDT, NOPROTECT, NOBROWNOUT, PUT, NOLVP

#use delay(clock=20000000)

#use rs232(BAUD=9600, XMIT=PIN_C6, RCV=PIN_C7, PARITY= N, BITS = 8, STREAM = BT, UART1)

#define POWER_PWM_PERIOD 499

//Definicao de pinos de entrada
#define F1 PIN_A0 //Sensor frontal da direita
#define F2 PIN_A1 //Sensor frontal da esquerda
#define L1 PIN_A3 //Sensor de linha da direita
#define L2 PIN_A4 //Sensor de linha da esquerda

//Definicao de pinos de saida

#define LED1  PIN_B6
#define LED2  PIN_B7
#define LED3  PIN_C4
#define LED4  PIN_C5

#define PWM_A PIN_B0 
#define PWM_B PIN_B2
#define AIN1  PIN_C2
#define AIN2  PIN_C3
#define BIN1  PIN_C1
#define BIN2  PIN_C0

//Definicao de constantes de controle

#define frente    1
#define atras     0
#define direita   1
#define esquerda  0

//Definicao de variaveis globais utilizadas no controle do robo

int16 leitura;      // variavel que recebe o valor digital lido diretamente de um sensor
int8 entrada = 0;   // variavel onde seus bits informam quais sensores foram acionados
int controle = 0;   // variavel para auxiliar no controle do robo via bluetooth
char  btRead = 'w'; // variavel para armazenar o caractere recebido via blutooth

//  'HEADERS' DAS FUNCOES UTILIZADAS

void lerDados(int8 canal, int8 bit);
void lerFrontal();
void lerLinha();
void moverMotor(int1 motor, int16 velocidade, int1 sentido);
void pararMotor(int1 motor);
void getStrategy();
void buscaEstrela();
void buscaRotativa(int sentido);
void trataSensores();




//  FUNCOES BASICAS PARA CONTROLE DO ROBO

/*
    lerDados(int8 canal, int8 bit);
    
    esta funcao le o canal analogico informado em 'canal', armazena o valor da
    conversao em 'leitura' para posterior comparacao. Altera o valor do bit informado
    em 'bit' de acordo com o valor de leitura.
*/

void lerDados(int8 canal, int8 bit){
    switch (canal){ //prepara o canal adequado para leitura
           case 1: set_adc_channel(1); break; // frontal direita
           case 2: set_adc_channel(2); break; // frontal esquerda
           case 3: set_adc_channel(3); break; // linha direita
           case 4: set_adc_channel(4); break; // linha esquerda
    }

    delay_us(10);

    leitura = read_adc();
    
    if (canal == 1 || canal == 2){               // Sensores frontais
        if(leitura < 800) bit_set(entrada, bit); // Sensor frontal acionado -> valor 1 no bit correspondente
        else bit_clear(entrada, bit);            // Caso contrario -> valor 0 no bit correspondente
    }
    if (canal == 3 || canal == 4){               // Sensores de linha
        if(leitura < 350) bit_set(entrada, bit); // Sensor de linha acionado -> valor 1 no bit correspondente
        else bit_clear(entrada, bit);            // Caso contrario -> valor 0 no bit correspondente
    }
}


/*
    void lerFrontal();

    esta funcao direciona a leitura aos sensores frontais
*/

void lerFrontal(){
    lerDados(1, 0); //sensor da direita
    lerDados(2, 1); //sensor da esquerda
}


/*
    void lerLinha();

    esta funcao direciona a leitura aos sensores de linha
*/

void lerLinha(){
    lerDados(3, 2); //sensor da direita
    lerDados(4, 3); //sensor da esquerda
}

/*
    void moverMotor(int1 motor, int16 velocidade, int1 sentido);
    
    Esta funcao e' responsavel pelo controle de ambos os motores.
    'motor'      -> variavel booleana que informa qual motor a ser controlado(direita/esquerda)*
    'velocidade' -> variavel inteira com valores compreendidos entre 0 e 2000
    'sentido'    -> variavel booleana que informa qual o sentido de rotacao(frente/atras)*
    
    *ver definicoes de constantes de controle
*/

void moverMotor(int1 motor, int16 velocidade, int1 sentido){

    if(motor){ //motor = 1(ou da direita)
        switch(sentido){ 
            case frente: output_high(BIN1);
                         output_low(BIN2);
                         break;
            
            case atras:  output_low(BIN1);
                         output_high(BIN2);
                         break;
        }
        set_power_pwm2_duty(velocidade);
    }

    else{ //motor = 0(ou da esquerda)
    
        switch(sentido){
            case frente: output_high(AIN1);
                         output_low(AIN2);
                         break;

            case atras:  output_low(AIN1);
                         output_high(AIN2);
                         break;
        }
        set_power_pwm0_duty(velocidade);
    }
}

/*
    void pararMotor(int1 motor);

    Funcao simples, responsavel por parar o motor informado em 'motor'
*/

void pararMotor(int1 motor){
    if(motor){
        output_low(BIN1);
        output_low(BIN2);
        set_power_pwm2_duty(0);
    }
    else{
        output_low(AIN1);
        output_low(AIN2);
        set_power_pwm0_duty(0);
    }
}

void getStrategy(){
    switch(controle){
        case 1: buscaRotativa(direita);
        //case 2: buscaEstrela();
        //case 3: varredura();
    }
}

void buscaEstrela(){
    //TODO
}

void varredura(){
    //TODO
}

void buscaRotativa(int sentido){
    if (sentido == 1){
        moverMotor(direita, 500, atras);
        moverMotor(esquerda, 500, frente);
    }
    else if(sentido == 0){
        moverMotor(esquerda, 500, frente);
        moverMotor(direita, 500, atras);
    }
}

void trataSensores(){
    lerFrontal();
    lerLinha();
    switch(entrada){// TODO: mudar de switch para if, com funções para cada comportamento padrao
        /* 
        
            bits da entrada: 7654 3210
            
            onde cada bit corresponde a um sensor especifico:
            
            bit 0: sensor frontal da direita
            bit 1: sensor frontal da esquerda 
            bit 2: sensor de linha da direita
            bit 3: sensor de linha da esquerda
            demais bits: nao utilizados
            
            Vale notar que 1 corresponde a sensor disparado, 0 c.c.

        */
        case  0: //0000 0000 
                 getStrategy(); 
                 break;

        case  1: //0000 0001
                 moverMotor(direita, 1500, atras);
                 moverMotor(esquerda, 1500, frente);
                 break;
                 
        case  2: //0000 0010
                 moverMotor(esquerda, 1500, atras);
                 moverMotor(direita, 1500, frente);
                 break;

        case  3: //0000 0011
                 moverMotor(direita, 1500, frente);
                 moverMotor(esquerda, 1500, frente);
                 break;

        case  4: //0000 0100
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(direita, 1600, atras);
                 moverMotor(esquerda, 1300, atras);
                 delay_ms(100);
                 break;

        case  5: //0000 0101
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(direita, 1600, atras);
                 moverMotor(esquerda, 1300, atras);
                 delay_ms(100);
                 break;

        case  6: //0000 0110
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(direita, 1600, atras);
                 moverMotor(esquerda, 1300, atras);
                 delay_ms(100);
                 break;

        case  7: //0000 0111
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(direita, 1600, atras);
                 moverMotor(esquerda, 1300, atras);
                 delay_ms(100);
                 break;

        case  8: //0000 1000
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(esquerda, 1600, atras);
                 moverMotor(direita, 1300, atras);
                 delay_ms(100);
                 break;

        case  9: //0000 1001
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(esquerda, 1600, atras);
                 moverMotor(direita, 1300, atras);
                 delay_ms(100);
                 break;

        case 10: //0000 1010
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(esquerda, 1600, atras);
                 moverMotor(direita, 1300, atras);
                 delay_ms(100);
                 break;

        case 11: //0000 1011
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(esquerda, 1600, atras);
                 moverMotor(direita, 1300, atras);
                 delay_ms(100);
                 break;

        case 12: //0000 1100
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(esquerda, 1500, atras);
                 moverMotor(direita, 1500, atras);
                 delay_ms(100);
                 break;

        case 13: //0000 1101
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(esquerda, 1500, atras);
                 moverMotor(direita, 1500, atras);
                 delay_ms(100);
                 break;

        case 14: //0000 1110
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(esquerda, 1500, atras);
                 moverMotor(direita, 1500, atras);
                 delay_ms(100);
                 break;

        case 15: //0000 1111
                 pararMotor(direita);
                 pararMotor(esquerda);
                 moverMotor(esquerda, 1500, atras);
                 moverMotor(direita, 1500, atras);
                 delay_ms(100);
                 break;
    }
}

#INT_RDA
void RDA_isr(void){

    btRead = getc();

    if      (btRead == 'a') controle = 1;
    else if (btRead == 'b') controle = 0;
}

void main(){

    set_tris_a(0b11111111);
    set_tris_b(0b00000000);
    set_tris_c(0b10000000);

    setup_adc_ports(ALL_ANALOG);
    setup_adc(ADC_CLOCK_INTERNAL);

    setup_power_pwm_pins(PWM_BOTH_ON, PWM_BOTH_ON, PWM_OFF, PWM_OFF);
    setup_power_pwm(PWM_FREE_RUN,1,0,POWER_PWM_PERIOD,0,1,33);

    enable_interrupts(INT_RDA);
    enable_interrupts(global);

    while(controle == 0){
    }

    //disable_interrupts(INT_RDA);
    //disable_interrupts(global);
    //delay_ms(5000);

    while (true){
        while (controle != 0){
        /*
        lerFrontal();
            switch (entrada){
                case 0: pararMotor(direita);
                        pararMotor(esquerda);
                        break;
                        
                case 1: moverMotor(esquerda, 500, frente);
                        moverMotor(direita, 500, atras);
                        break;
                        
                case 2: moverMotor(direita, 500, frente);
                        moverMotor(esquerda, 500, atras);
                        break;

                case 3: pararMotor(direita);
                        pararMotor(esquerda);
                        break;
            }
        */

            trataSensores();

        }

        while (controle == 0){
            pararMotor(direita);
            pararMotor(esquerda);
        }
    }
}
