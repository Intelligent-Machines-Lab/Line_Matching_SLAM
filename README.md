# Line_Matching_SLAM
## Introdução

**Autora**: Luciana Araujo Lemos (luarlemos96@hotmail.com), ITA.

**Orientador**: Cairo L. Nascimento Jr. (cairo@ita.br), ITA.

**Título**: Localização e Mapeamento Simultâneos usando Scanner a Laser e Correspondência entre as Características do Ambiente

Este software está disponível no seguinte site:
> **Github**: https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM

A Dissertação de Mestrado e outras publicações de Luciana Araujo Lemos estão disponíveis em:
> **Github**: https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/My_Publications

## Pré-requisitos

Software utilizado nesse trabalho:
- MATLAB 2021b
- ROS Noetic
- Gazebo 9.0

## Estrutura

As pastas contidas nesse repositório possuem os códigos-fonte e estão prontos para serem baixados e utilizados uma vez que os programas pré requisitos estejam instalados.

### **Workspaces**

A Workspace ROS utilizada no PC está presente em: [workspace do pc](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/workspaces/pc_ws). Para ser identificada pelo ROS, o comando **catkin_make** deve ser executado após a cópia da workspace.

A Workspace ROS utilizada no robô está presente em: [workspace do robô](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/workspaces/isis_ws).

## Organização dos arquivos
- Arquivos do Arduino no robô
  - Pasta: [Arduino](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/Arduino)
- Softwares Desenvolvidos:
  - Calibração do Scanner a Laser (LiDAR)
    - Pasta: [Calibration](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/Calibration)
  - Detecção de Cantos
    - Pasta: [Corner_Detection](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/Corner_Detection)
  - Junção de Segmentos (Line Merge)
    - Pasta: [Line_Merge](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/Line_Merge)
  - RANSAC
    - Pasta: [RANSAC](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/RANSAC)
  - ROS no MATLAB
    - Pasta: [ROS_MATLAB](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/ROS_MATLAB)
  - Experimentos realizados no Projeto usando SLAM 
    - Pasta: [SLAM_FM_LC](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/SLAM_FM_LC)
- My_Publications
  - PDF da dissertação: [Dissertacao_ITA_Luciana_Lemos_Versao_Final.pdf](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/blob/main/My_Publications/Dissertacao_ITA_Luciana_Lemos_Versao_Final.pdf)
  - Pasta: [Presentation](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/My_Publications/Presentation)

## Reprodução do trabalho

Os dados coletados por meio do ROS no MATLAB foram obtidos pelos scripts desenvolvidos: `teste_ros4.m` e `ros_isis.m`, localizados na pasta **ROS_MATLAB**. O primeiro foi utilizado para coletar os dados dos experimentos simulados e o segundo para o experimento real no robô móvel.

Os dados tratados pelo RANSAC foram obtidos pelos scripts desenvolvidos no MATLAB: `teste_Ransac12.m` e `ransac_isis5.m`, localizados na pasta **RANSAC**. O primeiro foi utilizado para gerar os dados dos segmentos para cada postura analisada nos experimentos simulados e o segundo para o experimento real no robô móvel.

Para reproduzir os experimentos realizados no trabalho é executado o script principal para realizar o SLAM com correspondências de características em cada um dos experimentos desenvolvido no MATLAB: `Slam_fm_LC.m`, localizado nas pastas dos experimentos em: [SLAM_FM_LC](https://github.com/Intelligent-Machines-Lab/Line_Matching_SLAM/tree/main/SLAM_FM_LC). Estas pastas contêm os arquivos **.mat** que foram os dados coletados nos experimentos. 
