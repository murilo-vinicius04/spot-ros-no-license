# Modificações Técnicas no Spot Hardware Interface

## Resumo

Este documento detalha todas as modificações técnicas realizadas no arquivo `spot_hardware_interface.cpp` para permitir o funcionamento do controle de juntas do robô Spot **sem a necessidade de licença de streaming**. As modificações substituem o sistema de streaming de baixo nível por uma abordagem de polling básico que utiliza APIs de alto nível do Boston Dynamics SDK.

## Contexto

O sistema original requeria uma **licença de streaming** (Joint Control License) para:
- Controle de juntas em tempo real
- Streaming de estado do robô em alta frequência
- Comandos de baixo nível via `RobotCommandStreamingClient`

As modificações implementadas permitem:
- ✅ Controle básico de juntas do braço usando APIs de alto nível
- ✅ Feedback de estado do robô via polling (10Hz)
- ✅ Funcionamento sem licença de streaming
- ✅ Compatibilidade com ros2_control

## Modificações Implementadas

### 1. Adição de Includes para APIs de Alto Nível

**Arquivo:** `spot_hardware_interface.cpp` (linhas 45-48)

```cpp
// ADDED: Includes for high-level arm commands (works without streaming license)
#include "bosdyn/api/arm_command.pb.h"
#include "bosdyn/api/robot_command.pb.h"
#include "bosdyn/api/synchronized_command.pb.h"
```

**Propósito:** Adiciona suporte para comandos de alto nível que não requerem licença de streaming.

### 2. Implementação do Loop de Polling Básico

**Arquivo:** `spot_hardware_interface.cpp` (linhas 745-815)

```cpp
// ADDED: Basic state polling loop that doesn't require streaming license
void basic_state_loop(std::stop_token stop_token, ::bosdyn::client::RobotStateClient* stateClient,
                      std::shared_ptr<StateStreamingHandler> state_handler, std::chrono::milliseconds poll_interval)
```

**Características:**
- **Frequência:** 10Hz (100ms de intervalo)
- **Cliente:** `RobotStateClient` (básico, sem streaming)
- **Dados coletados:**
  - Estados das juntas (posição, velocidade, torque)
  - Estados de contato dos pés
  - Dados básicos de IMU
  - Transformações de pose do corpo

**Vantagens:**
- Não requer licença de streaming
- Compatível com hardware interface padrão
- Dados suficientes para controle básico

### 3. Modificação do Sistema de Streaming de Estado

**Arquivo:** `spot_hardware_interface.cpp` (linhas 818-837)

```cpp
bool SpotHardware::start_state_stream(StateHandler&& state_policy) {
  // MODIFIED: Use basic robot state client instead of streaming client (no license required)
  auto robot_state_client_resp = robot_->EnsureServiceClient<::bosdyn::client::RobotStateClient>();
  // ...
  // Start polling loop using basic state client instead of streaming
  state_thread_ = std::jthread(&spot_hardware_interface::basic_state_loop, basic_state_client_, 
                               state_streaming_handler_, std::chrono::milliseconds(100)); // 10Hz polling
}
```

**Mudanças principais:**
- Substitui `RobotStateStreamingClient` por `RobotStateClient`
- Inicia thread de polling em vez de streaming
- Mantém compatibilidade com interface existente

### 4. Modificação do Sistema de Comandos

**Arquivo:** `spot_hardware_interface.cpp` (linhas 868-897)

```cpp
bool SpotHardware::start_command_stream() {
  // MODIFIED: Skip joint control mode activation (requires streaming license)
  RCLCPP_WARN(rclcpp::get_logger("SpotHardware"), "Skipping joint control mode activation (requires streaming license)");
  
  // NOTE: Skip RobotCommandStreamingClient creation (requires low-level license)
  command_stream_service_ = nullptr;
  
  // MODIFIED: Skip streaming request setup (requires low-level license)
  // joint_request_ setup removed since we're not using streaming commands
}
```

**Mudanças principais:**
- Remove criação de `RobotCommandStreamingClient`
- Pula ativação do modo de controle de juntas
- Mantém `RobotCommandClient` básico para comandos de alto nível

### 5. Implementação de Comandos de Alto Nível

**Arquivo:** `spot_hardware_interface.cpp` (linhas 900-950)

```cpp
void SpotHardware::send_command(const JointCommands& joint_commands) {
  // MODIFIED: Send high-level commands without streaming license (using basic robot API)
  
  // Rate limiting to avoid API overload (2Hz max for basic API)
  if (time_since_last < std::chrono::milliseconds(500)) { // 2Hz max
    return;
  }
  
  // Detect if this is arm control or body control
  if (has_arm && joint_commands.position.size() >= 19) {
    send_arm_command_basic(joint_commands);
  } else if (has_body) {
    send_body_command_basic(joint_commands);
  }
}
```

**Características:**
- **Rate limiting:** Máximo 2Hz para evitar sobrecarga da API
- **Detecção automática:** Diferencia comandos do braço vs. corpo
- **Fallback seguro:** Funciona mesmo sem streaming

### 6. Comandos Específicos do Braço

**Arquivo:** `spot_hardware_interface.cpp` (linhas 952-1010)

```cpp
void SpotHardware::send_arm_command_basic(const JointCommands& joint_commands) {
  // Extract arm joint positions (last 7 joints: sh0, sh1, el0, el1, wr0, wr1, f1x)
  const size_t arm_start = joint_commands.position.size() - 7;
  
  // Create arm joint command
  auto arm_command = ::bosdyn::api::RobotCommand();
  auto* synchronized_command = arm_command.mutable_synchronized_command();
  auto* arm_sync_command = synchronized_command->mutable_arm_command();
  auto* arm_joint_command = arm_sync_command->mutable_arm_joint_move_command();
  
  // Set joint positions using DoubleValue setters (only first 6 joints, skip gripper f1x)
  auto* positions = point->mutable_position();
  positions->mutable_sh0()->set_value(joint_commands.position[arm_start + 0]);
  positions->mutable_sh1()->set_value(joint_commands.position[arm_start + 1]);
  // ... (outros joints)
}
```

**Características:**
- **Juntas controladas:** sh0, sh1, el0, el1, wr0, wr1 (6 juntas do braço)
- **Protocolo:** `ArmJointMoveCommand` via API de alto nível
- **Trajetória:** Ponto único com tempo de 1 segundo
- **Compatibilidade:** Funciona sem licença de streaming

### 7. Adição de Novos Membros na Classe

**Arquivo:** `spot_hardware_interface.hpp` (linhas 357-364)

```cpp
// ADDED: Support for basic state client (no streaming license)
std::shared_ptr<::bosdyn::client::RobotStateClient> basic_state_client_{nullptr};

// ADDED: New command functions for basic API
void send_arm_command_basic(const JointCommands& joint_commands);
void send_body_command_basic(const JointCommands& joint_commands);

// ADDED: Basic state loop function
void basic_state_loop(std::stop_token stop_token, ::bosdyn::client::RobotStateClient* stateClient,
                      std::shared_ptr<StateStreamingHandler> state_handler, std::chrono::milliseconds poll_interval);
```

## Limitações e Considerações

### Limitações Técnicas

1. **Frequência reduzida:** 
   - Estado: 10Hz (vs. 100Hz+ com streaming)
   - Comandos: 2Hz (vs. 100Hz+ com streaming)

2. **Controle limitado:**
   - Apenas comandos de alto nível para o braço
   - Controle de corpo limitado a comandos básicos de mobilidade
   - Sem controle de baixo nível das pernas

3. **Latência aumentada:**
   - Polling introduz latência adicional
   - Menos responsivo que streaming

### Vantagens

1. **Sem licença de streaming:**
   - Funciona com licença básica do Spot
   - Reduz custos de licenciamento

2. **Compatibilidade mantida:**
   - Interface ros2_control inalterada
   - Funciona com controladores existentes

3. **Robustez:**
   - Menos dependente de conectividade de rede
   - Fallback automático para modo básico

## Testes Realizados

### Ambiente de Teste
- **Robô:** Spot com IP 192.168.80.3
- **Credenciais:** admin/spotadmin2017
- **Container:** Docker com spot_ros2_control

### Funcionalidades Testadas

1. ✅ **Inicialização do hardware interface**
   - Autenticação bem-sucedida
   - Conexão com cliente básico de estado
   - Início do loop de polling

2. ✅ **Feedback de estado**
   - Recepção de dados das juntas
   - Estados de contato dos pés
   - Dados de IMU básicos

3. ✅ **Comandos do braço**
   - Envio de comandos via API de alto nível
   - Controle das 6 juntas principais do braço
   - Rate limiting funcionando corretamente

4. ✅ **Integração com ros2_control**
   - Controladores carregam corretamente
   - Interface de comando funcional
   - Publicação de estados via tópicos

### Problemas Resolvidos

1. **"Invalid vision_t_body!" errors:**
   - **Causa:** Dados de pose inválidos/zero durante inicialização
   - **Solução:** Loop de polling básico fornece dados válidos

2. **Conflitos de lease:**
   - **Causa:** Múltiplos clientes tentando controlar o robô
   - **Solução:** Uso de APIs de alto nível sem conflito

3. **Falhas de autenticação:**
   - **Causa:** Tentativas de usar streaming sem licença
   - **Solução:** Fallback para cliente básico

## Compilação e Uso

### Compilação
```bash
cd /path/to/spot_ws
colcon build --packages-select spot_hardware_interface
source install/setup.bash
```

### Uso
```bash
# Iniciar o sistema completo
ros2 launch spot_ros2_control spot_ros2_control.launch.py \
  hostname:=192.168.80.3 \
  username:=admin \
  password:=spotadmin2017 \
  controllable:=true
```

### Comandos de Teste
```bash
# Testar comando do braço
ros2 topic pub /forward_position_controller/commands std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.3, 1.0, 0.0, 0.0, 0.0, 0.0]"

# Verificar estado
ros2 topic echo /joint_states
```

## Arquivos Modificados

1. **`src/spot_ros2/spot_hardware_interface/src/spot_hardware_interface.cpp`**
   - Implementação principal das modificações
   - ~200 linhas adicionadas/modificadas

2. **`src/spot_ros2/spot_hardware_interface/include/spot_hardware_interface/spot_hardware_interface.hpp`**
   - Declarações de novos métodos e membros
   - ~10 linhas adicionadas

## Conclusão

As modificações implementadas permitem o uso efetivo do Spot Hardware Interface **sem a necessidade de licença de streaming**, mantendo funcionalidade essencial para:

- Controle básico do braço robótico
- Feedback de estado completo
- Integração com ros2_control
- Operação estável e confiável

O sistema resultante é adequado para aplicações que não requerem controle de alta frequência, oferecendo uma alternativa econômica e funcional ao sistema de streaming completo. 