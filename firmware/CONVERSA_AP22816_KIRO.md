# Resumo da conversa para spec (Kiro)

## Objetivo discutido
Avaliar se o componente AP22816BKEWT-7 (load switch) pode ser substituido por MOSFET P AO3407, com foco principal no controle de alimentacao dos sensores Hall TLI493D-A2B6.

## Contexto do projeto
- Projeto: firmware de um mouse 3D com 3 sensores Hall e anel de LEDs.
- Placa: sensor_board (arquivo Eagle em `pcbs/src/sensor_board.txt`).
- Firmware: controle de power rails em `firmware/src/controllers/SensorController.cpp`.

## Topologia identificada no esquema
- Sensores Hall:
  - MAG1, MAG2, MAG3 = TLI493D-A2B6
  - Cada sensor tem chave de alimentacao dedicada via AP22816 (U1, U2, U3)
  - Rails de sensores em 3V3
- LEDs:
  - D1..D8 = SK6812
  - Alimentacao dos LEDs chaveada por AP22816 U4 (rail de 5V)
- Sinais de enable:
  - `!EN_MAG1`, `!EN_MAG2`, `!EN_MAG3`, `!EN_LEDS`
  - Ativo-baixo (LOW liga, HIGH desliga)
- I2C:
  - Sensores compartilham SDA/SCL
  - Pull-ups via resistores

## O que o AP22816BKEWT-7 faz
- E um load switch (chave eletronica de alimentacao) high-side.
- Pinos relevantes:
  - IN: entrada de alimentacao
  - OUT: saida chaveada
  - `!EN`: enable ativo-baixo
  - GND
  - FLG: flag de falha
- Vantagens do AP22816:
  - Controle digital simples via GPIO
  - Integracao de protecoes (ex.: sobrecorrente/termica)
  - Substitui circuito discreto maior

## Pergunta: GPIO do XIAO poderia alimentar o TLI493D direto?
Conclusao: nao recomendado.
- Corrente pode ficar no limite de GPIO dependendo da configuracao.
- Principal motivo funcional: os 3 TLI493D sobem com endereco I2C padrao igual, e o firmware precisa ligar um por vez para reatribuir endereco sem conflito no barramento.
- Por isso o projeto usa chaveamento individual por sensor.

## Pergunta: para substituir AP22816, precisa ser MOSFET P?
Conclusao: para high-side simples, sim, P-channel e a abordagem direta.
- N-channel em high-side exigiria gate acima de VDD (driver adicional), aumentando complexidade.

## Avaliacao especifica: AO3407 no lugar do AP22816

### U1/U2/U3 (sensores em 3V3)
Viavel com AO3407.
- Gate em LOW -> Vgs negativo -> liga
- Gate em HIGH (3V3) -> Vgs ~0 -> desliga
- Funciona bem para chavear VDD dos sensores.

### U4 (LEDs em 5V)
Nao e drop-in direto com GPIO 3V3.
- Para desligar PMOS high-side no rail de 5V, gate precisa ir proximo de 5V.
- GPIO 3V3 sozinho pode nao garantir OFF completo.
- Exige estagio adicional (ex.: transistor NPN/NMOS para puxar gate a 5V via pull-up), e possivel inversao de logica.

## Implicacoes no firmware
Arquivo atual usa semantica ativa-baixa para as linhas de switch.
- Em `SensorController.cpp`, hoje:
  - `powerOff(pin)` escreve LOW
  - `powerOn(pin)` escreve HIGH
- Se trocar U1/U2/U3 por PMOS discreto com gate direto no GPIO (sem inversor), a logica tende a inverter:
  - LOW liga
  - HIGH desliga
- Portanto, pode ser necessario ajustar `powerOn`/`powerOff` e testar a sequencia de boot/endereco I2C.

## Nota de projeto importante (gate default)
Para PMOS high-side, default seguro de OFF geralmente e gate puxado para SOURCE (pull-up no rail), nao pull-down para GND.
- Pull-down no gate tende a ligar o PMOS por padrao.
- Isso afeta comportamento no boot antes do firmware configurar GPIO.

## Recomendacao consolidada
1. Se o objetivo for apenas os sensores Hall (U1/U2/U3):
   - AO3407 e viavel.
   - Definir rede de gate com estado default coerente de boot.
   - Validar no firmware a logica de ON/OFF e a sequencia de atribuicao de enderecos I2C.
2. Manter U4 (LED 5V) como load switch integrado (AP22816 ou equivalente) para evitar complexidade extra.
3. Se quiser AO3407 tambem em U4:
   - Prever estagio de acionamento adicional para OFF real em 5V.
   - Revisar logica e protecoes (perde parte das protecoes integradas do AP22816).

## Itens para transformar em spec no Kiro
- Escopo:
  - Substituir apenas U1/U2/U3 ou tambem U4?
- Requisitos eletricos:
  - Corrente por rail, queda de tensao aceitavel, estado default no boot.
- Requisitos de firmware:
  - Tabela de verdade ON/OFF por pin.
  - Sequencia de power-up e endereco I2C dos 3 sensores.
- Validacao:
  - Teste de boot frio
  - Teste de detecao I2C sem conflito
  - Teste de ruido/estabilidade de leitura
  - Teste de consumo
- Riscos:
  - Logica invertida de enable
  - Estado incorreto no boot
  - OFF incompleto no rail de 5V

## Decisao tecnica sugerida
- Caminho de menor risco: manter AP22816 em U4 (LEDs 5V) e permitir AO3407 apenas em U1/U2/U3 (sensores 3V3), com ajuste de firmware e pull network correta no gate.
