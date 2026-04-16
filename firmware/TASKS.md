# Motion Processing — Implementation Tasks

> Tracking file for SPEC.md implementation.  
> Updated: 2026-04-15

---

## Phase 0 — Infraestrutura de testes

- [x] Criar ambiente `[env:native]` no platformio.ini
- [x] Criar shim Arduino.h para compilação desktop
- [x] Escrever testes de caracterização (baseline do comportamento atual)
- [x] Escrever testes unitários (math utilities, edge cases, simetria)
- [x] Validar: 30/30 testes passando

---

## Phase 1 — Per-axis deadzone (Spec §2.3)

Menor mudança, menor risco. Estabelece o padrão de trabalho.

- [x] Adicionar `DEADZONE[6]` em `Config.h`
- [x] Remover `DEAD_T`, `DEAD_R`, e `axisBaseDead()` 
- [x] Substituir referências em `MotionController::compute()` por `Config::DEADZONE[i]`
- [x] Atualizar testes: ajustar `test_deadzone_silence` para verificar por eixo
- [x] Adicionar teste: deadzones assimétricas (ex: Tx=10, Ry=30)
- [x] Validar: todos os testes passam (34/34)
- [x] Validar: build `seeed_xiao_rp2040` compila sem erros

---

## Phase 2 — Matriz de transformação 6×9 (Spec §2.1)

Substitui as fórmulas hard-coded. Bloco fundamental para tudo que segue.

- [x] Calcular e definir `TRANSFORM[6][9]` em `Config.h` com os coeficientes atuais
- [x] Implementar `MotionController::transform()`
- [x] Refatorar `compute()` para usar `transform()` no lugar das fórmulas inline
- [x] Remover variáveis de posição dos sensores (mag1PosX, etc.) — agora embutidas na matriz
- [x] Validar: testes de caracterização passam sem alteração (mesma saída — 39/39)
- [x] Adicionar teste: verificação de propriedades da matriz (somas, dimensões, sparsity)
- [x] Validar: build `seeed_xiao_rp2040` compila sem erros (Flash -40 bytes)

---

## Phase 3 — Compensação cross-axis 6×6 (Spec §2.2)

Depende da Phase 2 (opera sobre a saída do transform).

- [x] Adicionar `COMP[6][6]` (identidade) em `Config.h`
- [x] Implementar `MotionController::compensate()`
- [x] Inserir `compensate()` no pipeline de `compute()`, após `transform()`
- [x] Validar: testes de caracterização passam (identidade = sem mudança — 41/41)
- [x] Adicionar teste: identidade é passthrough (verifica cada elemento)
- [x] Adicionar teste: saída com identidade == baseline characterization
- [ ] Atualizar testes de bleed: marcar como "bleed reduzido" quando COMP for tuned
- [x] Validar: build `seeed_xiao_rp2040` compila sem erros (+216 bytes Flash)

---

## Phase 4 — Curva de resposta não-linear (Spec §2.4)

Independente das phases 2-3, mas fica melhor depois.

- [x] Adicionar `RESPONSE_LINEARITY` em `Config.h` (default = 1.0 para manter comportamento)
- [x] Implementar `MotionController::responseCurve()`
- [x] Inserir `responseCurve()` no pipeline, após gain/sign e antes do filtro
- [x] Validar: com `linearity=1.0`, testes de caracterização passam (46/46)
- [x] Adicionar teste: `linearity=1.0` → saída inalterada
- [x] Adicionar teste: simetria (f(-x) == -f(x))
- [x] Adicionar teste: zero passthrough
- [x] Adicionar teste: clamp at AXIS_LIMIT
- [x] Validar: build `seeed_xiao_rp2040` compila sem erros (+104 bytes Flash)

---

## Phase 5 — Filtro biquad (Spec §2.5)

Mais complexo, deve ser o último.

- [x] Adicionar `BiquadState` struct em `MotionController.h`
- [x] Adicionar `FILTER_FREQ_HZ`, `FILTER_Q`, e `#define FILTER_BIQUAD` em `Config.h`
- [x] Implementar `MotionController::biquadLP()` (Direct Form II Transposed, Audio EQ Cookbook)
- [x] Condicionar compilação com `#ifdef FILTER_BIQUAD` (manter lowpass como fallback)
- [x] Atualizar `reset()` para zerar `BiquadState`
- [x] Ajustar testes de filtro para tolerâncias do biquad (overshoot ≤5% Butterworth)
- [x] Adicionar teste: step response bounded overshoot com Q=0.707
- [x] Adicionar teste: convergência ao valor estacionário
- [x] Adicionar teste: retorno a zero com deadzone
- [x] Adicionar teste: estabilidade numérica com dt variável
- [x] Adicionar teste: reset limpa estado do biquad
- [x] Validar: 51/51 testes passam (`pio test -e native`)
- [x] Validar: build `seeed_xiao_rp2040` compila sem erros (+664 bytes Flash, +48 bytes RAM)

---

## Phase 6 — Integração e documentação

- [x] Atualizar `firmware/README.md` com novo pipeline e parâmetros de configuração
- [x] Atualizar `SPEC.md` status para "Implemented"
- [ ] Teste end-to-end no hardware real (subjetivo: Fusion 360 / FreeCAD)
- [ ] Tuning dos coeficientes COMP e RESPONSE_LINEARITY com dispositivo físico
