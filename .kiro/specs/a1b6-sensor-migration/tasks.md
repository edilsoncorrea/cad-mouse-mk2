# Plano de Implementação: Migração de Sensor A1B6

## Visão Geral

Migrar o `SensorController` do sensor TLI493D-A2B6 (geração 2) para o TLI493D-A1B6 (geração 1). As alterações são localizadas em dois arquivos (`SensorController.h` e `SensorController.cpp`) e envolvem: troca de tipo, ajuste de endereços I2C, remoção de `setSensitivity()` e adição de comentários de documentação.

## Tarefas

- [x] 1. Atualizar o header SensorController.h
  - [x] 1.1 Trocar o tipo dos membros de sensor de `TLx493D_A2B6` para `TLx493D_A1B6`
    - Alterar as 3 declarações de membros privados: `mag1Sensor_`, `mag2Sensor_`, `mag3Sensor_`
    - De: `ifx::tlx493d::TLx493D_A2B6`
    - Para: `ifx::tlx493d::TLx493D_A1B6`
    - O header `TLx493D_inc.hpp` já inclui ambas as classes, nenhum include adicional necessário
    - _Requisitos: 1.1, 1.3_

- [x] 2. Atualizar o construtor e método begin() em SensorController.cpp
  - [x] 2.1 Alterar o endereço padrão no construtor
    - Trocar `TLx493D_IIC_ADDR_A0_e` por `TLx493D_IIC_ADDR_A4_e` nas 3 inicializações da lista de membros
    - O endereço A4 (0x3E / 0x1F em 7-bit) é o padrão do A1B6 com SDA baixo no boot neste hardware
    - _Requisitos: 1.2, 2.5_

  - [x] 2.2 Atualizar os enums de reatribuição de endereço no begin()
    - MAG1: trocar `TLx493D_IIC_ADDR_A2_e` por `TLx493D_IIC_ADDR_A6_e` (0x1E / 0x0F)
    - MAG2: trocar `TLx493D_IIC_ADDR_A1_e` por `TLx493D_IIC_ADDR_A5_e` (0x36 / 0x1B)
    - MAG3 permanece no endereço padrão A4 (sem chamada a `setIICAddress()`)
    - _Requisitos: 2.1, 2.2, 2.3, 2.4, 2.5, 2.6_

  - [x] 2.3 Remover as chamadas a setSensitivity()
    - Remover as 3 chamadas `mag*Sensor_.setSensitivity(TLx493D_EXTRA_SHORT_RANGE_e)` no begin()
    - A1B6 não suporta setSensitivity() — retorna false e emite warning
    - _Requisitos: 3.1_

- [x] 3. Adicionar comentários de documentação
  - [x] 3.1 Adicionar bloco de documentação no topo de SensorController.cpp
    - Documentar que os sensores são TLI493D-A1B6 (geração 1)
    - Incluir mapa de endereços I2C utilizado (grupo SDA-baixo: A4=0x3E, A5=0x36, A6=0x1E, A7=0x16)
    - Documentar que `setSensitivity()` não é suportado no A1B6 (full range, 0.098 mT/LSB)
    - Documentar diferenças principais em relação ao A2B6
    - _Requisitos: 3.2, 8.1, 8.2_

  - [x] 3.2 Adicionar comentários inline junto às constantes de endereço no begin()
    - Anotar cada chamada `setIICAddress()` com o endereço 8-bit e 7-bit correspondente
    - Anotar que MAG3 permanece no endereço padrão A4
    - _Requisitos: 8.2_

- [x] 4. Checkpoint — Verificar compilação
  - Executar `pio run -e seeed_xiao_rp2040` e garantir build limpo sem erros ou warnings
  - Ensure all tests pass, ask the user if questions arise.
  - _Requisitos: 1.1, 1.2, 1.3, 2.1–2.6, 3.1, 3.2, 4.1–4.3, 5.1, 5.2, 6.1, 6.2, 7.1, 7.2, 8.1, 8.2_

## Notas

- Apenas 2 arquivos são modificados: `SensorController.h` e `SensorController.cpp`
- A interface pública do `SensorController` permanece inalterada — nenhum outro módulo precisa de ajuste
- O método `readRaw()` não requer alterações (API `getMagneticFieldAndTemperature()` é idêntica entre gerações)
- A lógica de calibração e power switch permanece intacta
- Property-based testing não se aplica — as operações são chamadas diretas a hardware I2C
- O único teste automatizável é a compilação (`pio run`); validação completa requer hardware real
- Testes manuais no hardware (I2C scan, leitura magnética, calibração, operação do mouse) devem ser realizados após o flash
