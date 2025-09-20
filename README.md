# Challenge FIAP e Otis

## 1EMR 2025 - Bacharelado em Engenharia Mecatrônica

### <span style="color:dodgerblue;">Breno Martins</span> - RM563685 \| <span style="color:dodgerblue;">Henrique Cruz</span> - RM564586 \| <span style="color:dodgerblue;">Leonardo Eiji</span> - RM562934



**DESAFIO**

*Criar um projeto mecatrônico com sistema portátil de monitoramento inteligente aplicado ao ajuste e identificação de falhas de elevadores.*



Estruturas propostas:

1. Protótipo de Elevador em MDF com trilhos contendo Sensores Magnéticos (Tipo Reed Switch), Temperatura, Corrente Elétrica e Vibração. Cabine é movimentada por um motor DC de 12v em conjunto a uma ponte H. Todo o protótipo é controlado por microcontrolador ESP32.
2. Protótipo portátil com sensores de alcance a laser para medições. A movimentação se dá através de 4 motores de passo modelo NEMA17 em conjunto aos drivers A4988.
3. Dashboard alimentado por dados em nuvem em servidores AWS que recebem informações via telemetria (MQTT) de ambos os protótipos.




Componentes

```
//Controlador
ESP32 com Módulo Expansor (Shield)
Fonte de alimentação 12v 3A

//Sensores
4 unidades de Sensores magnéticos Reed Switch (Intelbras Xas)
Sensor DHT11 para temperatura e humidade
Sensor ACS712 (20A) para medição de corrente elétrica
Sensor MPU-6050 Acelerômetro e giroscópio para medição de vibração

//Motores
Motor DC 12v 100rpm com caixa de redução
Ponte H L298H
Fonte de Alimentação 12v 3A
```




