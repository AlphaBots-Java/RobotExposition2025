# Code Review

## 1. Construtor incorreto

O método `public void ElevatorSubsystem()` **não é um construtor**. Em Java, construtores **não possuem tipo de retorno**. Assim, a configuração do `SparkMax` nunca é aplicada, pois o método não é chamado automaticamente. O construtor correto deve ser:

```java
public ElevatorSubsystem() { ... }
```

## 2. Mistura de dois PIDs

O código configura um PID interno do SparkMax usando:

```java
config.closedLoop.pid(0.01, 0.0, 0);
```

Mas o controle aplicado é externo usando `PIDController` do WPILib:

```java
elevatorMotor.set(pidController.calculate(...));
```

Isso mistura dois modelos de controle. É recomendável escolher um:

* **Closed-loop interno do SparkMax** via `setReference()`, ou
* **PID externo do WPILib**, removendo o PID interno.

## 3. Encoder não utilizado

O campo:

```java
Encoder caEncoder;
```

É declarado mas **nunca inicializado** e **nunca usado**. Deve ser removido ou corretamente implementado.

## 4. Imports não utilizados

Os imports:

* `ArmFeedforward`
* `PS5Controller`

Não são usados no código e devem ser removidos para evitar confusão.

## 5. Exposição indevida do motor

```java
public SparkMax cansparkMax;
```

Deveria ser:

```java
private final SparkMax elevatorMotor;
```

Boas práticas recomendam encapsular completamente os componentes do subsistema.

## 6. Nomenclatura de métodos

Métodos como:

```java
public void SetPoint()
public void UpdateSetPoint()
```

Não seguem convenções Java. Correção recomendada:

```java
public void updateSetpoint()
public void applyControl()
```

## 7. Controle sem clamp e sem lógica robusta

A lógica de controle em `SetPoint()` não possui:

* Limites (clamp) de saída
* Feedforward (importante para elevadores)
* Tratamento seguro de extremos de posição

## 8. Conversões do encoder

```java
positionConversionFactor(1)
velocityConversionFactor(1)
```

Sem conversão real para unidades físicas (metros, graus), dificultando tuning e análise.

## 9. Estilo e organização

O pacote deveria seguir o padrão Java:

```
frc.robot.subsystems
```

Não `Subsystems` com inicial maiúscula.

---

### **Resumo geral**

Este subsistema apresenta problemas funcionais (construtor errado), inconsistência na lógica de controle (dois PIDs diferentes), falta de encapsulamento, nomenclatura fora do padrão e código morto. Uma reestruturação clara é recomendada, escolhendo um único método de controle (PID interno ou externo) e garantindo que a configuração do motor seja realmente aplicada no construtor.

## Review: AnguladorSubsystem

### 1. Construtor incorreto novamente

Assim como no subsistema anterior, o método:

```java
public void AnguladorSubsystem() {
```

Não é um construtor. Isso significa que **nenhuma configuração do SparkMax é aplicada automaticamente**. O construtor correto deveria ser:

```java
public AnguladorSubsystem() {
    ...
}
```

Sem isso, o motor roda com parâmetros default e todo o controle (PID, idleMode, feedback) é ignorado.

### 2. Mistura de closed-loop interno com PID + feedforward externo

O código configura PID interno:

```java
config.closedLoop.pid(1.5, 0.5, 1);
```

Mas no controle, usa **PID externo do WPILib** + **feedforward externo**:

```java
cansparkMaxAng.set(-feedforward.calculate(...) + pidControllerAng.calculate(...));
```

Isto gera três problemas:

* PID interno nunca é usado.
* PID externo e feedforward externo não estão sincronizados com a configuração do Spark.
* Controle híbrido dificulta tuning e causa comportamento instável.

Deve-se escolher um único método:

* **Closed-loop interno** (setReference + FF interno), OU
* **PID externo + FF externo**, removendo `config.closedLoop.pid()`.

### 3. Problemas graves na cinemática do feedforward

Trechos como:

```java
(m_setpoint.position + 30/360) * 2 * Math.PI
```

`30/360` é divisão inteira em Java → resultado = **0**, não 0.0833.
Correção:

```java
(30.0 / 360.0)
```

Isso afeta todo o feedforward. O controle está sendo calculado com valores incorretos.

Além disso:

* Multiplicar por `2 * Math.PI` assume que `m_setpoint.position` está em rotações, mas não há garantia.
* Não há unidade clara (graus? radianos? rotações?).
* Divisão fixa por 18 é um "magic number" sem explicação.

### 4. CANcoder está sendo usado diretamente sem conversão

```java
canCoder.getPosition().getValueAsDouble()
```

Esse valor normalmente está em **rotações** (ou 0–1), dependendo da configuração do Phoenix.
Mas o PID espera a mesma unidade usada pelo setpoint do TrapezoidProfile — hoje isso não bate.

Falta:

* Conversão para graus ou radianos.
* Offset aplicado corretamente (`angleOffsetDeg`).

### 5. Encoder caEncoder não é usado

Mesma situação do outro subsistema — variável declarada e esquecida.

### 6. Variáveis públicas expostas

```java
public SparkMax cansparkMaxAng;
public CANcoder canCoder;
```

Componentes de hardware não devem ser públicos.
Use encapsulamento adequado:

```java
private final SparkMax angMotor;
private final CANcoder absEncoder;
```

### 7. Uso questionável do PS5Controller dentro do subsistema

```java
private PS5Controller cont = new PS5Controller(0);
```

O subsistema **não deveria ter controle direto da entrada humana**.
Isso deve estar nos Commands.

### 8. SmartDashboard dentro do subsistema

Embora funcional, em projetos maiores isso polui o subsistema. Recomenda-se centralizar dashboards em classes utilitárias ou Commands.

### 9. Magic numbers espalhados

Trechos como:

* `+30/360`
* `/18`
* `0.02` (período do profile)
* `0.6`, `0.4` (constraints)

Devem ser constantes nomeadas:

```java
private static final double kLoopPeriod = 0.02;
```

### 10. TrapezoidProfile implementado corretamente, mas inicialização parcial

O uso de `m_goal` e `m_setpoint` está ok, porém:

```java
m_timer.restart();
```

`m_timer` não é usado no perfil, tornando o reset inútil.

### 11. Unidades do TrapezoidProfile não definidas

O profile assume:

* posição: **?** graus? radianos? rotações?
* velocidade: mesma unidade / s

Mas o feedforward assume radianos.
O PID assume valor cru do encoder.
O CANcoder retorna rotações.
**Nada conversa com nada.**

### 12. Estrutura sugerida

O ideal seria:

* Definir unidade — **graus** ou **radianos**
* Normalizar tudo:

```java
double currentAngleRad = canCoder.getAbsolutePosition() * 2π;
double ff = feedforward.calculate(currentAngleRad, m_setpoint.velocity);
double pid = pid.calculate(currentAngleRad, m_setpoint.position);
angMotor.set(ff + pid);
```

E remover closed-loop interno do Spark.

---

### **Resumo geral deste subsistema**

* Construtor incorreto evita que a configuração seja aplicada.
* Controle está extremamente inconsistente: unidades diferentes, divisão inteira errada, soma com magic numbers, feedforward com cálculos incorretos e PID duplo.
* Estrutura precisa de uma revisão completa focada em *unidades físicas*, *encapsulamento* e *coerência entre os métodos de contr
