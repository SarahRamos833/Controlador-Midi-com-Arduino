////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////ESSA PROGRAMAÇÃO É UM COMPILADO DE OUTRAS PROGRAMAÇÕES QUE ENCONTREI NO GITHUB/////////////////////////////////////
//////////////A PARTE DO LDR SÓ ABRE O CANAL MIDI E NÃO FECHA, FORA QUE É UMA GAMBIARRA DENTRO DESSE CODIGO (SORRY)/////////////////////////////
/////////////////////////////////////////////////BOA SORTE NOS SEUS PROJETOS////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////BY SARAH//////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ARDUINO

// Remova o comentário desta linha para habilitar as saídas correspondentes ao MIDI Fighter 
#define MIDI_FIGHTER

// define para definir e limpar bits de registro
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Mapeamento MIDI retirado de http://www.nortonmusic.com/midi_cc.html
#define MIDI_CC_MODULATION 0x01
#define MIDI_CC_BREATH 0x02
#define MIDI_CC_VOLUME 0x07
#define MIDI_CC_BALANCE 0x08
#define MIDI_CC_PAN 0x0A
#define MIDI_CC_EXPRESSION 0x0B
#define MIDI_CC_EFFECT1 0x0C
#define MIDI_CC_EFFECT2 0x0D

#define MIDI_CC_GENERAL1 0x0E
#define MIDI_CC_GENERAL2 0x0F
#define MIDI_CC_GENERAL3 0x10
#define MIDI_CC_GENERAL4 0x11
#define MIDI_CC_GENERAL5 0x12
#define MIDI_CC_GENERAL6 0x13
#define MIDI_CC_GENERAL7 0x14
#define MIDI_CC_GENERAL8 0x15
#define MIDI_CC_GENERAL9 0x16
#define MIDI_CC_GENERAL10 0x17
#define MIDI_CC_GENERAL11 0x18
#define MIDI_CC_GENERAL12 0x19
#define MIDI_CC_GENERAL13 0x1A
#define MIDI_CC_GENERAL14 0x1B
#define MIDI_CC_GENERAL15 0x1C
#define MIDI_CC_GENERAL16 0x1D
#define MIDI_CC_GENERAL17 0x1E
#define MIDI_CC_GENERAL18 0x1F

#define MIDI_CC_GENERAL1_FINE 0x2E
#define MIDI_CC_GENERAL2_FINE 0x2F
#define MIDI_CC_GENERAL3_FINE 0x30
#define MIDI_CC_GENERAL4_FINE 0x31
#define MIDI_CC_GENERAL5_FINE 0x32
#define MIDI_CC_GENERAL6_FINE 0x33
#define MIDI_CC_GENERAL7_FINE 0x34
#define MIDI_CC_GENERAL8_FINE 0x35
#define MIDI_CC_GENERAL9_FINE 0x36
#define MIDI_CC_GENERAL10_FINE 0x37
#define MIDI_CC_GENERAL11_FINE 0x38
#define MIDI_CC_GENERAL12_FINE 0x39
#define MIDI_CC_GENERAL13_FINE 0x3A
#define MIDI_CC_GENERAL14_FINE 0x3B
#define MIDI_CC_GENERAL15_FINE 0x3C
#define MIDI_CC_GENERAL16_FINE 0x3D
#define MIDI_CC_GENERAL17_FINE 0x3E
#define MIDI_CC_GENERAL18_FINE 0x3F

#define MIDI_CC_SUSTAIN 0x40
#define MIDI_CC_REVERB 0x5B
#define MIDI_CC_CHORUS 0x5D
#define MIDI_CC_CONTROL_OFF 0x79
#define MIDI_CC_NOTES_OFF 0x78

#define NOTE_C0 0x00 // 0
#define NOTE_C1 0x12 // 18
#define NOTE_C2 0x24 // 36
#define NOTE_C3 0x36 // 36

//////////////////////////////////////////////////////MUDAR AQUI//////////////////////////////////////////////
  // Numero de portas digitais - botões
  #define NUM_DI 11
  // Numero de portas analogicas - potenciometros 
  #define NUM_AI 2
////////////////////////////////////////////////////PARAR DE MUDAR O CODIGO///////////////////////////////////

#if defined(MIDI_FIGHTER) && defined(ARDUINO)
  #define MIDI_CHANNEL 3
  // Primeira nota, começando do botão inferior esquerdo
  #define NOTE NOTE_C3
  // Ao mapear para um MIDI Fighter, precisamos pular uma linha de botões. Defina de 0 a 3 para definir qual linha pular.
   // As linhas são ordenadas de baixo para cima (igual ao layout dos botões do MIDI Fighter).
  #define SKIP_ROW 2
  // Esta ordem de pino corresponde ao botão inferior esquerdo sendo zero, aumentando em um conforme nos movemos da esquerda para a direita, de baixo para cima
  // Este tamanho de array deve corresponder a NUM_DI acima.
  #define DIGITAL_PIN_ORDER 13, 10, 11, 12, 6, 7, 8, 9, 3, 4, 5
#else
  #define MIDI_CHANNEL 1
  // Primeira nota, começando com o botão superior esquerdo
  #define NOTE NOTE_C0
  // Esta ordem de pino corresponde ao botão superior esquerdo sendo zero, aumentando em um conforme nos movemos da esquerda para a direita, de cima para baixo
  // Este tamanho de array deve corresponder a NUM_DI acima.
    //ordem dos pinos digitais
    #define DIGITAL_PIN_ORDER  3, 4, 5, 6, 7, 8, 9, 10, 11, 12,13
#endif
  //ordem dos pinos analoicos
#define ANALOGUE_PIN_ORDER A0,A1
  
int PIN_D6;
  #define LED_PIN 13

#define MIDI_CC MIDI_CC_GENERAL1

// Comente esta linha para desativar a lógica de debounce do botão.
#define DEBOUNCE
// Duração do tempo de debounce em milissegundos
#define DEBOUNCE_LENGTH 2

// Comente esta linha para desativar a filtragem analógica
#define ANALOGUE_FILTER
// Um botão ou movimento do controle deslizante deve inicialmente exceder esse valor para ser reconhecido como uma entrada. Observe que é
// para um valor MIDI de 7 bits (0-127).

#define FILTER_AMOUNT 2

// O tempo limite é em microssegundos
#define ANALOGUE_INPUT_CHANGE_TIMEOUT 250000

// Matriz contendo um mapeamento de pinos digitais para índice de canal.
byte digitalInputMapping[NUM_DI] = {DIGITAL_PIN_ORDER};

// Array contendo um mapeamento de pinos analógicos para índice de canal. Este tamanho de matriz deve corresponder a NUM_AI acima.
byte analogueInputMapping[NUM_AI] = {ANALOGUE_PIN_ORDER};

// Contém o estado atual das entradas digitais.
byte digitalInputs[NUM_DI];
// Contém o valor atual das entradas analógicas.
byte analogueInputs[NUM_AI];

// Variável para conter leituras digitais temporárias, usada para lógica de debounce.
byte tempDigitalInput;
// Variável para manter valores analógicos temporários, usado para lógica de filtragem analógica.
byte tempAnalogueInput;

// Pré-aloque o índice do loop for para que não continuemos realocando-o para cada iteração do programa.
byte i = 0;
byte digitalOffset = 0;
// Variável para manter a diferença entre os valores de entrada analógica atuais e novos.
byte analogueDiff = 0;
// Isso é usado como um sinalizador para indicar que uma entrada analógica está mudando.
boolean analogueInputChanging[NUM_AI];
// Horário em que a entrada analógica foi movida pela última vez
unsigned long analogueInputTimer[NUM_AI];


void setup()
{ 
   pinMode(2,OUTPUT);
    Serial.begin(115200);
  // Inicialize cada canal de entrada digital.
  for (i = 0; i < NUM_DI; i++)
  {
    // Defina a direção do pino para entrada.
    pinMode(digitalInputMapping[i], INPUT);

    // Não habilite o resistor pullup em LED_PIN, pois o LED e o resistor sempre o puxarão para baixo, significando que a entrada não funcionará.
     // Em vez disso, um resistor suspenso externo deve ser usado no LED_PIN.
     // NOTA: Isso fará com que toda a lógica alta / baixa para LED_PIN seja invertida.
    if (digitalInputMapping[i] != LED_PIN)
    {
      // Habilite o resistor pull-up. Esta chamada deve vir após a chamada pinMode acima.
      digitalWrite(digitalInputMapping[i], HIGH);
    }
    
    //Inicialize o estado digital com uma leitura do pino de entrada.
    digitalInputs[i] = digitalRead(digitalInputMapping[i]);
  }
  
  // nicialize cada canal de entrada analógica.
  for (i = 0; i < NUM_AI; i++)
  {
    // Defina a direção do pino para entrada.
    pinMode(analogueInputMapping[i], INPUT);
    
    // Inicialize o valor analógico com uma leitura do pino de entrada.
    analogueInputs[i] = analogRead(analogueInputMapping[i]);
    
    // Suponha que nenhuma entrada analógica esteja ativa
    analogueInputChanging[i] = false;
    analogueInputTimer[i] = 0;
  }
}


void loop()
{ 
/////////////////////////////////////////////////////////////////////////botoes//////////////////////////////////////////////////////////////////////////////////////
  for (i = 0; i < NUM_DI; i++)
  {
    #ifdef MIDI_FIGHTER
      if (i >= SKIP_ROW * 4)
      {
        digitalOffset = i + 4;
      }
      else
      {
    #endif
 
    digitalOffset = i;
    
    #ifdef MIDI_FIGHTER
      }
    #endif
    
    // Lê o estado atual da entrada digital e armazena-o temporariamente.
    tempDigitalInput = digitalRead(digitalInputMapping[i]);
    
    // Verifique se o último estado é diferente do estado atual.
    if (digitalInputs[i] != tempDigitalInput)
    {
      #ifdef DEBOUNCE
      // Aguarde um curto período de tempo e faça uma segunda leitura do pino de entrada.
      delay(DEBOUNCE_LENGTH);
      // Se a segunda leitura for igual à leitura inicial, assuma que deve ser verdadeira.
      if (tempDigitalInput == digitalRead(digitalInputMapping[i]))
      {
      #endif
        // Registre o novo estado da entrada digital.
        digitalInputs[i] = tempDigitalInput;
        
        // Movido de HIGH para LOW (botão pressionado)
        if (digitalInputs[i] == 0)
        {
          // Todas as entradas digitais usam resistores pullup, exceto LED_PIN, então a lógica é invertida
          if (digitalInputMapping[i] != LED_PIN)
          {
            noteOn(MIDI_CHANNEL, NOTE + digitalOffset, 0x7F); // Channel 1, middle C, maximum velocity
          }
          else
          {
            noteOff(MIDI_CHANNEL, NOTE + digitalOffset); // Channel 1, middle C
          }
        }
        // Movido de LOW para HIGH (botão liberado)
        else
        {
          // Todas as entradas digitais usam resistores pullup, exceto LED_PIN, então a lógica é invertida
          if (digitalInputMapping[i] != LED_PIN)
          {
            noteOff(MIDI_CHANNEL, NOTE + digitalOffset); // Channel 1, middle C
          }
          else
          {
            noteOn(MIDI_CHANNEL, NOTE + digitalOffset, 0x7F); // Channel 1, middle C, maximum velocity
          }
        }
      #ifdef DEBOUNCE
      }
      #endif
    }
  }
  
///////////////////////////////////////////////////////////////potenciometros///////////////////////////////////////////////////////////////////////////////////////
  for (i = 0; i < NUM_AI; i++)
  {
    //Leia o pino de entrada analógica, dividindo-o por 8, para que o valor ADC de 10 bits (0-1023) seja convertido em um valor MIDI de 7 bits (0-127).
    tempAnalogueInput = analogRead(analogueInputMapping[i]) / 8;
    
    #ifdef ANALOGUE_FILTER
    // Pegue o valor absoluto da diferença entre os valores atuais e novos
    analogueDiff = abs(tempAnalogueInput - analogueInputs[i]);
    // Continue apenas se o limite foi excedido ou a entrada já estava mudando
    if ((analogueDiff > 0 && analogueInputChanging[i] == true) || analogueDiff >= FILTER_AMOUNT)
    {
      // Somente reinicie o cronômetro se tivermos certeza de que a entrada não está "entre" um valor
       // ie. Foi movido mais de FILTER_AMOUNT
      if (analogueInputChanging[i] == false || analogueDiff >= FILTER_AMOUNT)
      {
        // Reinicializar a última vez que a entrada foi movida
        analogueInputTimer[i] = micros();
        
        
        // A entrada analógica está se movendo
        analogueInputChanging[i] = true;
      
      }
      else if (micros() - analogueInputTimer[i] > ANALOGUE_INPUT_CHANGE_TIMEOUT)
      {
        analogueInputChanging[i] = false;
      }
      
      // Apenas envie dados se soubermos que a entrada analógica está se movendo
      if (analogueInputChanging[i] == true)
      {
        // Registre o novo valor analógico
        analogueInputs[i] = tempAnalogueInput;
        
        // Envie o valor analógico para o MIDI CC geral (consulte as definições no início deste arquivo)
        controlChange(MIDI_CHANNEL, MIDI_CC + i, analogueInputs[i]);
        
      }
    }
    #else
    if (analogueInputs[i] != tempAnalogueInput)
    {
      // Registre o novo valor analógico
      analogueInputs[i] = tempAnalogueInput;
      
      // Envie o valor analógico para o MIDI CC geral (consulte as definições no início deste arquivo)
      controlChange(MIDI_CHANNEL, MIDI_CC + i, analogueInputs[i]);
    }
    #endif
  
  }
  
/////////////////////////////////////////////////////////////////////////////////LDR/////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////AVISO DE GAMBIARRA! SE NÃO FOR UTILIZAR LDR'S É SÓ REMOVER ESSA PARTE!///////////////////////////////////////////////////
    int ldr2[] = {A2,A3,A4,A5};
    int ldr[4];
    int note;
    for(int a=0;a<4;a++)
    {
    ldr[a]=analogRead(ldr2[a]);
    if(ldr[a]>950)
    {
       noteOn(MIDI_CHANNEL, NOTE + digitalOffset, 0x7F); // ENVIA SINAL MIDI
      Serial.println("ldr");
    }
    }
}
////////////////////////////////////////////////////////////////////////////////ENVIA SINAL MIDI//////////////////////////////////////////////////////////////////////////

// Envie uma nota MIDI na mensagem
void noteOn(byte channel, byte pitch, byte velocity)
{
  channel += 0x90 - 1;
  if (channel >= 0x90 && channel <= 0x9F)
  {
      Serial.write(channel);
      Serial.write(pitch);
      Serial.write(velocity);
      
  }
}
// Enviar uma mensagem MIDI note off
void noteOff(byte channel, byte pitch)
{
  channel += 0x80 - 1;
  if (channel >= 0x80 && channel <= 0x8F)
  {
      Serial.write(channel);
      Serial.write(pitch);
      Serial.write((byte)0x00);
      digitalWrite(2,HIGH);
      delay(100);
      digitalWrite(2,LOW);
  }
}
// Envie uma mensagem de alteração de controle de MIDI
void controlChange(byte channel, byte control, byte value)
{
  channel += 0xB0 - 1;
  if (channel >= 0xB0 && channel <= 0xBF)
  {
      Serial.write(channel);
      Serial.write(control);
      Serial.write(value);
      digitalWrite(2,HIGH);
      delay(50);
      digitalWrite(2,LOW);
  }
}
