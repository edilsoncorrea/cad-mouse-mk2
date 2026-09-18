# Documento de Requisitos

## Introdução

Este documento especifica os requisitos para adaptar o firmware do mouse 3D (cad-mouse-mk2) de modo a suportar a substituição dos load switches AP22816 (U1, U2, U3) por MOSFETs P-channel AO3407 nos rails de alimentação 3V3 dos sensores Hall TLI493D-A2B6. O load switch AP22816 do rail de LEDs (U4, 5V) permanece inalterado. A mudança principal é a inversão da lógica de enable: com PMOS, gate LOW liga o sensor e gate HIGH desliga, o oposto do comportamento atual do firmware.

## Glossário

- **SensorController**: Classe C++ em `firmware/src/controllers/SensorController.cpp` responsável pelo controle de alimentação e leitura dos sensores Hall.
- **Config**: Namespace em `firmware/include/Config.h` que define pinos de hardware e constantes do projeto.
- **PowerRailController**: Abstração de firmware (a ser criada ou adaptada) que encapsula a lógica de ligar/desligar um rail de alimentação, considerando o tipo de componente (AP22816 ou PMOS).
- **Rail_Sensor**: Rail de alimentação 3V3 de um sensor Hall individual, chaveado por AO3407 PMOS (U1, U2 ou U3).
- **Rail_LED**: Rail de alimentação 5V dos LEDs SK6812, chaveado por AP22816 (U4).
- **PMOS**: MOSFET P-channel (AO3407) usado como chave high-side. Gate LOW = conduz (ON), gate HIGH = não conduz (OFF).
- **AP22816**: Load switch integrado com enable ativo-baixo. !EN LOW = saída ligada, !EN HIGH = saída desligada.
- **GPIO_Enable**: Pino de saída digital do MCU (RP2040) que controla o gate do PMOS ou o !EN do AP22816.
- **Sequência_I2C**: Procedimento de ligar sensores um a um para atribuição de endereços I2C únicos, evitando conflitos no barramento compartilhado.
- **Estado_Boot**: Nível lógico do GPIO_Enable antes do firmware configurar o pino como saída, determinado pelo pull-up/pull-down externo.

## Requisitos

### Requisito 1: Abstração de Lógica de Enable por Tipo de Componente

**User Story:** Como desenvolvedor de firmware, eu quero que o controle de ligar/desligar rails de alimentação abstraia a diferença de lógica entre AP22816 e PMOS, para que o restante do código não precise conhecer o tipo de componente em cada rail.

#### Critérios de Aceitação

1. THE PowerRailController SHALL fornecer operações `powerOn` e `powerOff` que aceitem a identificação do rail e produzam o nível lógico GPIO correto para o tipo de componente associado.
2. WHEN `powerOn` é chamado para um Rail_Sensor (PMOS AO3407), THE PowerRailController SHALL escrever LOW no GPIO_Enable correspondente.
3. WHEN `powerOff` é chamado para um Rail_Sensor (PMOS AO3407), THE PowerRailController SHALL escrever HIGH no GPIO_Enable correspondente.
4. WHEN `powerOn` é chamado para o Rail_LED (AP22816), THE PowerRailController SHALL escrever HIGH no GPIO_Enable correspondente (mantendo a lógica do firmware original).
5. WHEN `powerOff` é chamado para o Rail_LED (AP22816), THE PowerRailController SHALL escrever LOW no GPIO_Enable correspondente (mantendo a lógica do firmware original).
6. THE PowerRailController SHALL manter uma tabela de mapeamento entre cada rail e o tipo de componente (PMOS ou AP22816) configurável em tempo de compilação.

### Requisito 2: Estado Seguro de Boot para Rails de Sensores PMOS

**User Story:** Como desenvolvedor de firmware, eu quero que todos os sensores Hall estejam desligados por padrão no boot (antes do firmware configurar GPIOs), para que não haja conflito de endereço I2C durante a inicialização.

#### Critérios de Aceitação

1. WHEN o MCU inicia e o firmware configura os pinos GPIO_Enable dos Rail_Sensor como saída, THE SensorController SHALL escrever imediatamente o nível lógico de OFF (HIGH para PMOS) em cada pino antes de qualquer operação I2C.
2. THE Config SHALL documentar em comentário que o hardware externo deve possuir pull-up no gate do PMOS para o rail 3V3 (SOURCE), garantindo estado OFF antes do firmware assumir controle.
3. WHILE os pinos GPIO_Enable dos Rail_Sensor ainda não foram configurados como saída pelo firmware, THE Estado_Boot SHALL manter os sensores desligados por meio do pull-up externo no gate do PMOS.

### Requisito 3: Sequência de Power-Up e Atribuição de Endereços I2C

**User Story:** Como desenvolvedor de firmware, eu quero que os sensores Hall sejam ligados um a um com atribuição de endereço I2C único antes de ligar o próximo, para que não haja conflito de endereço no barramento compartilhado.

#### Critérios de Aceitação

1. WHEN o SensorController executa a inicialização, THE SensorController SHALL ligar o Rail_Sensor do MAG1, atribuir um endereço I2C único ao MAG1, e aguardar a confirmação antes de ligar o Rail_Sensor do MAG2.
2. WHEN o Rail_Sensor do MAG2 é ligado, THE SensorController SHALL atribuir um endereço I2C único ao MAG2 e aguardar a confirmação antes de ligar o Rail_Sensor do MAG3.
3. WHEN o Rail_Sensor do MAG3 é ligado, THE SensorController SHALL atribuir um endereço I2C único ao MAG3.
4. IF dois ou mais sensores estiverem ligados simultaneamente com o mesmo endereço I2C padrão, THEN THE SensorController SHALL detectar a condição de conflito e reportar erro via serial.
5. WHEN cada sensor é ligado, THE SensorController SHALL aguardar no mínimo 5 ms após a ativação do rail antes de iniciar comunicação I2C com o sensor.

### Requisito 4: Compatibilidade com Rail de LEDs AP22816

**User Story:** Como desenvolvedor de firmware, eu quero que o controle do rail de LEDs (U4, AP22816) continue funcionando com a lógica original ativo-baixo, para que a substituição dos load switches dos sensores não afete o funcionamento dos LEDs.

#### Critérios de Aceitação

1. THE PowerRailController SHALL manter a lógica do firmware original (HIGH = ON, LOW = OFF) para o Rail_LED controlado pelo AP22816.
2. WHEN o firmware controla o Rail_LED, THE PowerRailController SHALL utilizar os mesmos níveis lógicos GPIO que o firmware original (HIGH para ligar, LOW para desligar).
3. THE Config SHALL manter o pino `PIN_LED_LS` (D1) com a mesma atribuição e comportamento do firmware original.

### Requisito 5: Configuração de Pinos e Mapeamento de Componentes

**User Story:** Como desenvolvedor de firmware, eu quero que o mapeamento entre pinos GPIO, rails e tipos de componente esteja centralizado no Config, para facilitar manutenção e eventuais mudanças futuras de hardware.

#### Critérios de Aceitação

1. THE Config SHALL definir constantes que associem cada pino de load switch (PIN_MAG1_LS, PIN_MAG2_LS, PIN_MAG3_LS, PIN_LED_LS) ao tipo de componente correspondente (PMOS ou AP22816).
2. THE Config SHALL manter os pinos de hardware existentes (PIN_MAG1_LS = D10, PIN_MAG2_LS = D9, PIN_MAG3_LS = D8, PIN_LED_LS = D1) inalterados.
3. WHEN um novo tipo de componente de chaveamento for adicionado no futuro, THE Config SHALL permitir a extensão do mapeamento sem alteração na lógica do PowerRailController.

### Requisito 6: Delay de Estabilização Pós-Ativação do Rail PMOS

**User Story:** Como desenvolvedor de firmware, eu quero que haja um delay configurável após ligar um rail PMOS antes de comunicar com o sensor, para garantir que a tensão esteja estável e o sensor esteja pronto para I2C.

#### Critérios de Aceitação

1. WHEN o PowerRailController liga um Rail_Sensor, THE SensorController SHALL aguardar um período de estabilização configurável (definido no Config) antes de iniciar comunicação I2C.
2. THE Config SHALL definir uma constante `POWER_ON_DELAY_MS` com valor padrão de 5 ms para o delay de estabilização pós-ativação de rail.
3. WHEN o PowerRailController liga o Rail_LED, THE SensorController SHALL aplicar o mesmo delay de estabilização configurável antes de utilizar os LEDs.

### Requisito 7: Validação de Leitura dos Sensores Após Inicialização

**User Story:** Como desenvolvedor de firmware, eu quero que o firmware valide que cada sensor responde corretamente após a inicialização com a nova lógica de enable, para detectar falhas de hardware ou configuração.

#### Critérios de Aceitação

1. WHEN um sensor Hall é inicializado e seu endereço I2C é atribuído, THE SensorController SHALL realizar uma leitura de teste para verificar que o sensor responde no endereço atribuído.
2. IF um sensor não responder à leitura de teste após a inicialização, THEN THE SensorController SHALL reportar o erro via serial indicando qual sensor falhou e em qual endereço I2C.
3. WHEN todos os três sensores são inicializados com sucesso, THE SensorController SHALL registrar via serial a confirmação de inicialização bem-sucedida com os endereços I2C atribuídos.

### Requisito 8: Tabela de Verdade de Níveis Lógicos GPIO

**User Story:** Como desenvolvedor de firmware, eu quero que a tabela de verdade dos níveis lógicos GPIO para cada tipo de componente esteja documentada e implementada de forma verificável, para evitar erros de inversão de lógica.

#### Critérios de Aceitação

1. THE PowerRailController SHALL implementar a seguinte tabela de verdade: para PMOS (AO3407), `powerOn` produz GPIO LOW e `powerOff` produz GPIO HIGH; para AP22816, `powerOn` produz GPIO HIGH e `powerOff` produz GPIO LOW (mantendo compatibilidade com o firmware original).
2. FOR ALL combinações de tipo de componente e operação (powerOn/powerOff), a conversão de operação lógica para nível GPIO SHALL ser determinística e verificável por teste unitário.
3. THE Config SHALL conter comentários documentando a tabela de verdade completa para referência de desenvolvedores.
