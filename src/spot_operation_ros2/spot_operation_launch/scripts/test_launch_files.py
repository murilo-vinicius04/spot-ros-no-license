#!/usr/bin/env python3

import os
import sys
import subprocess
import time
from pathlib import Path

def check_ros2_environment():
    """Verifica se o ambiente ROS2 está configurado."""
    print("🔍 Verificando ambiente ROS2...")
    
    try:
        result = subprocess.run(['ros2', '--version'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✅ ROS2 encontrado: {result.stdout.strip()}")
            return True
        else:
            print("❌ ROS2 não encontrado ou não configurado")
            return False
    except Exception as e:
        print(f"❌ Erro ao verificar ROS2: {e}")
        return False

def check_package_exists(package_name):
    """Verifica se um pacote ROS2 existe."""
    try:
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0 and package_name in result.stdout:
            print(f"✅ Pacote {package_name} encontrado")
            return True
        else:
            print(f"❌ Pacote {package_name} não encontrado")
            return False
    except Exception as e:
        print(f"❌ Erro ao verificar pacote {package_name}: {e}")
        return False

def check_launch_file_exists(launch_file):
    """Verifica se um launch file existe."""
    if os.path.exists(launch_file):
        print(f"✅ Launch file encontrado: {launch_file}")
        return True
    else:
        print(f"❌ Launch file não encontrado: {launch_file}")
        return False

def validate_launch_file(launch_file):
    """Valida um launch file usando ros2 launch --dry-run."""
    print(f"🔍 Validando launch file: {launch_file}")
    
    try:
        result = subprocess.run(['ros2', 'launch', '--dry-run', launch_file], 
                              capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            print(f"✅ Launch file válido: {launch_file}")
            return True
        else:
            print(f"❌ Launch file inválido: {launch_file}")
            print(f"Erro: {result.stderr}")
            return False
    except Exception as e:
        print(f"❌ Erro ao validar launch file {launch_file}: {e}")
        return False

def test_launch_file_execution(launch_file, timeout=10):
    """Testa a execução de um launch file por um tempo limitado."""
    print(f"🚀 Testando execução: {launch_file}")
    
    try:
        process = subprocess.Popen(['ros2', 'launch', launch_file], 
                                 stdout=subprocess.PIPE, 
                                 stderr=subprocess.PIPE,
                                 text=True)
        
        # Aguarda um tempo para verificar se inicia corretamente
        time.sleep(timeout)
        
        # Verifica se o processo ainda está rodando
        if process.poll() is None:
            print(f"✅ Launch file iniciou corretamente: {launch_file}")
            process.terminate()
            process.wait()
            return True
        else:
            stdout, stderr = process.communicate()
            print(f"❌ Launch file falhou: {launch_file}")
            print(f"Erro: {stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Erro ao executar launch file {launch_file}: {e}")
        return False

def main():
    """Função principal do teste."""
    print("🚀 Iniciando testes dos launch files do Spot Operation System")
    print("=" * 60)
    
    # Verificar ambiente ROS2
    if not check_ros2_environment():
        print("❌ Ambiente ROS2 não configurado. Execute 'source /opt/ros/humble/setup.bash'")
        return False
    
    # Obter diretório do workspace
    workspace_dir = Path(__file__).parent.parent.parent.parent
    launch_dir = workspace_dir / "src" / "spot_operation_ros2" / "spot_operation_launch" / "launch"
    
    if not launch_dir.exists():
        print(f"❌ Diretório de launch não encontrado: {launch_dir}")
        return False
    
    # Lista de launch files para testar
    launch_files = [
        "spot_operation_full.launch.py",
        "spot_operation_manual.launch.py", 
        "spot_operation_autonomous.launch.py",
        "spot_operation_debug.launch.py"
    ]
    
    # Verificar dependências dos pacotes
    required_packages = [
        "spot_operation_core",
        "spot_operation_vision", 
        "spot_operation_grasp",
        "spot_operation_control",
        "spot_operation_gesture",
        "spot_operation_launch"
    ]
    
    print("\n📦 Verificando dependências dos pacotes...")
    packages_ok = True
    for package in required_packages:
        if not check_package_exists(package):
            packages_ok = False
    
    if not packages_ok:
        print("❌ Alguns pacotes não foram encontrados. Execute 'colcon build' primeiro.")
        return False
    
    # Verificar existência dos launch files
    print("\n📁 Verificando existência dos launch files...")
    launch_files_ok = True
    for launch_file in launch_files:
        launch_path = launch_dir / launch_file
        if not check_launch_file_exists(str(launch_path)):
            launch_files_ok = False
    
    if not launch_files_ok:
        print("❌ Alguns launch files não foram encontrados.")
        return False
    
    # Validar launch files
    print("\n🔍 Validando launch files...")
    validation_ok = True
    for launch_file in launch_files:
        launch_path = launch_dir / launch_file
        if not validate_launch_file(str(launch_path)):
            validation_ok = False
    
    if not validation_ok:
        print("❌ Alguns launch files têm erros de sintaxe.")
        return False
    
    # Testar execução dos launch files (versão rápida)
    print("\n🚀 Testando execução dos launch files (versão rápida)...")
    execution_ok = True
    
    # Testar apenas o launch file principal por um tempo curto
    main_launch = launch_dir / "spot_operation_full.launch.py"
    if not test_launch_file_execution(str(main_launch), timeout=5):
        execution_ok = False
    
    # Resumo final
    print("\n" + "=" * 60)
    print("📊 RESUMO DOS TESTES")
    print("=" * 60)
    
    if packages_ok and launch_files_ok and validation_ok and execution_ok:
        print("✅ TODOS OS TESTES PASSARAM!")
        print("\n🎉 O sistema está pronto para uso!")
        print("\n📋 Próximos passos:")
        print("1. Configure o hardware (Spot, câmera, etc.)")
        print("2. Execute: ros2 launch spot_operation_launch spot_operation_full.launch.py")
        print("3. Use os parâmetros para personalizar o comportamento")
        return True
    else:
        print("❌ ALGUNS TESTES FALHARAM!")
        print("\n🔧 Ações recomendadas:")
        if not packages_ok:
            print("- Execute 'colcon build --packages-select spot_operation_ros2'")
        if not launch_files_ok:
            print("- Verifique se todos os arquivos foram criados corretamente")
        if not validation_ok:
            print("- Corrija os erros de sintaxe nos launch files")
        if not execution_ok:
            print("- Verifique as dependências e configurações")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 