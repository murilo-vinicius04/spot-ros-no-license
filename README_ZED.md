# ZED ROS 2 Wrapper - Instruções de Uso

## Configuração Realizada

O container ZED foi configurado para:
- Instalar automaticamente o ZED ROS 2 wrapper
- Configurar workspace em `/root/zed_workspace`
- Compilar o wrapper com `colcon build`
- Persistir dados no host em `./zed_workspace`

## Como Usar

### 1. Iniciar o container ZED
```bash
docker-compose up zed
```

### 2. Executar o ZED wrapper (opção fácil)
```bash
./run_zed.sh zed2    # Para ZED 2
./run_zed.sh zed2i   # Para ZED 2i
./run_zed.sh zedm    # Para ZED Mini
./run_zed.sh zedx    # Para ZED X
```

### 3. Executar manualmente dentro do container
```bash
# Entrar no container
docker exec -it zed-container bash

# Configurar ambiente
source /opt/ros/humble/setup.bash
source /root/zed_workspace/install/setup.bash

# Executar ZED wrapper
ros2 launch zed_wrapper zed2.launch.py
```

## Tópicos ROS 2 Disponíveis

Depois de executar o wrapper, você terá acesso aos seguintes tópicos:

- `/zed2/zed_node/rgb/image_rect_color` - Imagem RGB
- `/zed2/zed_node/depth/depth_registered` - Mapa de profundidade
- `/zed2/zed_node/point_cloud/cloud_registered` - Nuvem de pontos
- `/zed2/zed_node/pose` - Pose da câmera
- `/zed2/zed_node/odom` - Odometria visual
- `/zed2/zed_node/imu/data` - Dados IMU

## Configuração Adicional

Para configurar parâmetros específicos do ZED, edite os arquivos de configuração em:
`/root/zed_workspace/src/zed-ros2-wrapper/zed_wrapper/config/`

## Troubleshooting

### Container não encontra a câmera ZED
- Verifique se a câmera está conectada
- Confirme que o container tem acesso ao `/dev` com `privileged: true`

### Erro de compilação
- Execute novamente o container - ele irá recompilar automaticamente
- Verifique os logs: `docker logs zed-container`

### Performance baixa
- Certifique-se que o driver NVIDIA está instalado no host
- Verifique se o CUDA está funcionando: `nvidia-smi`
