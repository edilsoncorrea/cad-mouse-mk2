# Kiro Spec Draft - Substituicao de AP22816 por AO3407 (Sensores Hall)

## 1. Problem Statement
O design atual usa AP22816BKEWT-7 como load switch para energizacao seletiva de:
- U1/U2/U3: rails de 3V3 dos sensores Hall TLI493D-A2B6
- U4: rail de 5V dos LEDs

Existe interesse em substituir AP22816 por AO3407 (PMOS) para os sensores Hall, preservando:
- sequencia de power-up necessaria para atribuicao de enderecos I2C
- estabilidade no boot
- funcionamento confiavel no firmware

## 2. Goals
1. Permitir chaveamento individual de alimentacao de MAG1, MAG2 e MAG3 com AO3407.
2. Garantir sequencia de inicializacao sem conflito de endereco I2C.
3. Definir estado de boot previsivel (sem ligar sensores indevidamente antes do firmware).
4. Minimizar impacto no firmware e no layout.

## 3. Non-Goals
1. Nao substituir obrigatoriamente o switch de 5V dos LEDs (U4) nesta fase.
2. Nao alterar protocolo I2C dos sensores.
3. Nao refatorar arquitetura completa de controle de energia.

## 4. Scope
### In Scope
- Substituicao de U1/U2/U3 por PMOS AO3407.
- Revisao da logica de ON/OFF no firmware para os pinos de switch dos sensores.
- Definicao da rede de gate para default seguro no boot.

### Out of Scope
- Redesign completo da alimentacao dos LEDs (5V) com driver discreto adicional.
- Mudanca de MCU/plataforma.

## 5. Functional Requirements
1. O sistema deve permitir ligar apenas um sensor por vez durante a fase de atribuicao de endereco I2C.
2. O sistema deve manter sensores desligados no estado default de boot, conforme estrategia definida de gate pull.
3. O firmware deve expor funcoes de powerOn/powerOff coerentes com a polaridade final do hardware.
4. A leitura de MAG1/MAG2/MAG3 deve permanecer estavel apos a troca.

## 6. Electrical Requirements
1. U1/U2/U3 devem operar em high-side com PMOS AO3407 no rail de 3V3.
2. Deve haver rede de gate com valores definidos (ex.: resistor serie de gate e resistor de pull apropriado ao estado default).
3. O estado de OFF deve ser garantido no boot antes da configuracao dos GPIOs.
4. Queda de tensao no PMOS nao deve comprometer alimentacao do TLI493D.

## 7. Firmware Requirements
1. Revisar a polaridade efetiva de acionamento (active-low ou active-high) apos a mudanca de hardware.
2. Atualizar implementacoes de powerOn/powerOff no controlador de sensores.
3. Preservar a sequencia:
   - desliga todos
   - liga MAG1 e reendereca
   - liga MAG2 e reendereca
   - liga MAG3
4. Manter delays minimos necessarios para estabilizacao de rail e inicializacao do sensor.

## 8. Acceptance Criteria
1. Boot a frio inicia sem conflito de endereco I2C em 100 ciclos consecutivos.
2. Todos os 3 sensores respondem nos enderecos esperados apos inicializacao.
3. Leitura magnetica permanece funcional e sem regressao perceptivel de ruido.
4. Estados ON/OFF de cada sensor seguem corretamente os GPIOs definidos.
5. Nenhum sensor liga indevidamente durante reset/boot.

## 9. Test Plan
1. Teste de boot repetitivo (power cycle) com validacao dos enderecos I2C.
2. Teste funcional de ligar/desligar cada sensor individualmente.
3. Teste de leitura continua para verificar estabilidade e ruido.
4. Teste de comportamento durante reset do MCU.
5. Medicao de corrente por rail para confirmar estado OFF real.

## 10. Risks
1. Polaridade de gate invertida causando logica invertida no firmware.
2. Estado default incorreto no boot por escolha inadequada de pull no gate.
3. Substituicao de U4 (5V) sem driver adicional pode nao garantir OFF completo.

## 11. Decisions
1. Fase 1: substituir apenas U1/U2/U3 por AO3407.
2. Manter U4 (LED 5V) com load switch integrado nesta iteracao.
3. Atualizar firmware somente no necessario para refletir polaridade final de hardware.

## 12. Open Questions
1. Qual estado default de boot sera adotado oficialmente: sensores OFF ou ON?
2. Valores finais dos resistores de gate/pull para robustez EMC e transicao.
3. Necessidade futura de substituir U4 por solucao discreta com estagio auxiliar.

## 13. Implementation Checklist
1. Definir esquematico final de U1/U2/U3 com AO3407.
2. Revisar ERC/DRC do PCB.
3. Atualizar firmware de controle de energia dos sensores.
4. Executar plano de testes e registrar resultados.
5. Congelar decisao para U4 como backlog.
