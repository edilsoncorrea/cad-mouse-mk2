# Documento de Requisitos

## Introdução

Este documento especifica os requisitos para migrar o firmware do mouse 3D (SpaceMouse) do sensor Hall TLI493D-A2B6 (geração 2) para o TLI493D-A1B6 (geração 1). O projeto utiliza 3 sensores magnéticos idênticos conectados via I2C a uma placa Seeed XIAO RP2040, com a biblioteca Infineon XENSIV TLx493D.

A migração é necessária porque os sensores físicos montados na placa são A1B6 (gen 1), e não A2B6 (gen 2) como o firmware atual assume. As diferenças principais entre as gerações são: o tipo da classe C++ (`TLx493D_A2B6` → `TLx493D_A1B6`), o mapa de endereços I2C (completamente diferente), e a indisponibilidade de `setSensitivity()` na geração 1.

## Glossário

- **SensorController**: Classe do firmware responsável por inicializar, configurar e ler os 3 sensores magnéticos Hall. Definida em `SensorController.h` / `SensorController.cpp`.
- **Sensor_A1B6**: Instância da classe `ifx::tlx493d::TLx493D_A1B6` da biblioteca XENSIV, representando um sensor TLI493D-A1B6 (geração 1).
- **Load_Switch**: Chave eletrônica de alimentação (AP22816 ou equivalente) que controla individualmente a energia de cada sensor via GPIO. Pinos: D10 (MAG1), D9 (MAG2), D8 (MAG3).
- **Sequência_de_Endereçamento**: Procedimento de ligar os sensores um por vez via Load_Switch para atribuir endereços I2C distintos, evitando conflito no barramento (todos iniciam com o mesmo endereço padrão).
- **Endereço_Padrão_A1B6**: Endereço I2C padrão do A1B6 ao ligar com SDA alto: A0 = 0xBC (8-bit write) / 0x5E (7-bit). Com SDA baixo: A4 = 0x3E (8-bit) / 0x1F (7-bit).
- **Mapa_de_Endereços_Gen1**: Tabela de endereços I2C do A1B6 (SDA alto no boot): A0=0xBC, A1=0xB4, A2=0x9C, A3=0x94. (SDA baixo no boot): A4=0x3E, A5=0x36, A6=0x1E, A7=0x16. Todos em formato 8-bit write.
- **Config_Header**: Arquivo `firmware/include/Config.h` contendo constantes de configuração do firmware.

## Requisitos

### Requisito 1: Substituição do Tipo de Sensor

**User Story:** Como desenvolvedor do firmware, quero que as instâncias de sensor usem a classe correta para o hardware real (A1B6), para que a comunicação I2C e a leitura de dados funcionem corretamente.

#### Critérios de Aceitação

1. THE SensorController SHALL declarar os três membros de sensor como tipo `ifx::tlx493d::TLx493D_A1B6` em vez de `ifx::tlx493d::TLx493D_A2B6`.
2. THE SensorController SHALL construir cada Sensor_A1B6 com o objeto `Wire` e o endereço inicial `TLx493D_IIC_ADDR_A0_e` (endereço padrão da geração 1 com SDA alto no boot).
3. THE SensorController SHALL incluir os headers necessários para a classe `TLx493D_A1B6` através do header existente `TLx493D_inc.hpp`.

### Requisito 2: Sequência de Endereçamento I2C para Geração 1

**User Story:** Como desenvolvedor do firmware, quero que a sequência de power-on e atribuição de endereços I2C funcione com o mapa de endereços da geração 1, para que os 3 sensores operem simultaneamente sem conflito no barramento.

#### Critérios de Aceitação

1. THE SensorController SHALL desligar todos os três Load_Switch antes de iniciar a Sequência_de_Endereçamento.
2. WHEN o primeiro sensor (MAG1) é ligado via Load_Switch, THE SensorController SHALL chamar `begin()` no Sensor_A1B6 correspondente e em seguida reatribuir o endereço I2C usando `setIICAddress()` com um valor do enum `TLx493D_IICAddressType_t` que resulte em um endereço distinto do Endereço_Padrão_A1B6.
3. WHEN o segundo sensor (MAG2) é ligado via Load_Switch, THE SensorController SHALL chamar `begin()` no Sensor_A1B6 correspondente e em seguida reatribuir o endereço I2C para um valor distinto dos endereços já atribuídos.
4. WHEN o terceiro sensor (MAG3) é ligado via Load_Switch, THE SensorController SHALL chamar `begin()` no Sensor_A1B6 correspondente. O terceiro sensor pode permanecer no Endereço_Padrão_A1B6 ou receber um endereço distinto.
5. THE SensorController SHALL utilizar apenas endereços do Mapa_de_Endereços_Gen1 que correspondam ao estado do pino SDA no boot dos sensores (grupo A0–A3 para SDA alto, ou grupo A4–A7 para SDA baixo).
6. WHEN a Sequência_de_Endereçamento é concluída, THE SensorController SHALL garantir que os três sensores respondam em endereços I2C distintos.

### Requisito 3: Remoção da Chamada setSensitivity

**User Story:** Como desenvolvedor do firmware, quero que o código não chame funções indisponíveis no A1B6, para evitar falhas silenciosas e comportamento inesperado.

#### Critérios de Aceitação

1. THE SensorController SHALL remover todas as chamadas a `setSensitivity()` na inicialização dos sensores, pois a função retorna `false` e emite aviso de "feature not available" no A1B6.
2. IF uma funcionalidade equivalente a ajuste de sensibilidade for necessária no futuro, THEN THE SensorController SHALL documentar em comentário no código que o A1B6 opera apenas em faixa completa (full range) e não suporta `setSensitivity()`.

### Requisito 4: Leitura de Campo Magnético e Temperatura

**User Story:** Como desenvolvedor do firmware, quero que a leitura dos valores magnéticos (X, Y, Z) e temperatura continue funcionando após a migração, para que o mouse 3D opere normalmente.

#### Critérios de Aceitação

1. THE SensorController SHALL ler os valores de campo magnético e temperatura de cada Sensor_A1B6 usando `getMagneticFieldAndTemperature()`.
2. THE SensorController SHALL preencher o array de saída `out[9]` com os valores magnéticos dos 3 sensores na mesma ordem atual: [mag1x, mag1y, mag1z, mag2x, mag2y, mag2z, mag3x, mag3y, mag3z].
3. WHEN `getMagneticFieldAndTemperature()` retorna valores válidos, THE SensorController SHALL fornecer os dados ao pipeline de transformação sem alteração de formato ou escala.

### Requisito 5: Compatibilidade com Calibração Existente

**User Story:** Como desenvolvedor do firmware, quero que o processo de calibração (zero offset) continue funcionando sem modificações, para que a experiência do usuário não seja afetada.

#### Critérios de Aceitação

1. THE SensorController SHALL manter a interface pública de calibração (`beginCalibration()`, `updateCalibration()`, `calibrationDone()`, `baseline()`) sem alterações de assinatura ou comportamento.
2. THE SensorController SHALL calcular o baseline usando a mesma quantidade de amostras definida em `Config::ZERO_SAMPLES`.

### Requisito 6: Manutenção da Lógica de Power Switch

**User Story:** Como desenvolvedor do firmware, quero que o controle de liga/desliga dos sensores via Load_Switch continue usando a mesma lógica de GPIO, para que não haja impacto no hardware existente.

#### Critérios de Aceitação

1. THE SensorController SHALL manter os métodos `powerOn()` e `powerOff()` com a mesma semântica de GPIO atual (HIGH para ligar, LOW para desligar via load switch ativo-baixo com inversão no hardware).
2. THE SensorController SHALL utilizar os mesmos pinos de controle definidos em Config_Header: `PIN_MAG1_LS` (D10), `PIN_MAG2_LS` (D9), `PIN_MAG3_LS` (D8).

### Requisito 7: Configuração I2C do Barramento

**User Story:** Como desenvolvedor do firmware, quero que a configuração do barramento I2C (velocidade, inicialização) permaneça compatível com o A1B6, para garantir comunicação estável.

#### Critérios de Aceitação

1. THE SensorController SHALL inicializar o barramento I2C com `Wire.begin()` e `Wire.setClock(400000)` antes de iniciar a Sequência_de_Endereçamento.
2. WHEN o barramento I2C é inicializado, THE SensorController SHALL garantir que a velocidade de 400 kHz (Fast Mode) é compatível com o A1B6 conforme datasheet do sensor.

### Requisito 8: Documentação das Diferenças entre Gerações

**User Story:** Como desenvolvedor do firmware, quero que as diferenças relevantes entre A2B6 e A1B6 estejam documentadas no código, para facilitar manutenção futura.

#### Critérios de Aceitação

1. THE SensorController SHALL incluir comentários no código-fonte indicando que os sensores são TLI493D-A1B6 (geração 1) e listando as diferenças principais em relação ao A2B6: mapa de endereços I2C diferente, `setSensitivity()` indisponível, e fator de conversão magnético diferente (0.098 mT/LSB no gen 1).
2. THE SensorController SHALL documentar em comentário o mapa de endereços I2C utilizado (quais valores de enum correspondem a quais endereços físicos).
