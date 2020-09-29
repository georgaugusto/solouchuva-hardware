**<p align="center">(Ainda em produção - NÃO FINALIZADO)</p>**

<p align="center">
  <img src="https://raw.githubusercontent.com/georgaugusto/solouchuva/85adedbfe8c53a00d70f8e1ff746e8db709dfc8d/public/logoImg.svg" width="600px"/>
</p>

<p align="center">
  <a href="#Descrição">Descrição</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#Preview">Preview</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#Requisitos">Requisitos</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#Como-contribuir">Como contribuir</a>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
</p>

## Descrição
<p align="justify">
  Em consequência do constante crescimento do número de habitantes na Terra, haverá necessidade de aumentar significativamente a produção de alimentos, bem como combater as mais   variadas consequências desse crescimento, tais como as mudanças climáticas, a degradação do meio ambiente, a insegurança alimentar, a utilização de insumos de origem fóssil, o   aumento de doenças e a própria produtividade agrícola.
</p>
<p align="justify">
  Devido a esses problemas, a proposta desse trabalho é apresentar uma solução de baixo custo e simples utilização: Um protótipo de IoT para Smart Agriculture, com hardware       desenvolvido na placa NodeMCU, onde todos os dados coletados são enviados ao banco de dados na nuvem e, através desta, podem ser acessados em tempo real.  Tais dados serão       consumidos pela aplicação WebApp, desenvolvida no NodeJS e ReactJS, e, após, apresentados ao usuário.
</p>
<p align="justify">
  O protótipo supramencionado atua com foco no monitoramento meteorológico e é capaz de realizar previsões em tempo real, com mobilidade, e contemplando, ainda, a possibilidade   futura de que uma cultura desenvolva algum tipo de doença e a estimativa de produção e de crescimento por meio de deduções, a partir de dados, ações ou informações previamente   apresentadas, auxiliando, assim, a tomada de decisão do usuário.
</p>

Para ver a **describe**, clique aqui: [SolouChuva Describe](https://github.com/georgaugusto/solouchuva)</br>
Para ver a **api**, clique aqui: [SolouChuva Rest API](https://github.com/georgaugusto/solouchuva-backend)</br>
Para ver a **web client**, clique aqui: [SolouChuva Web](https://github.com/georgaugusto/solouchuva-frontend)

Para ver a **aplicação** rodando, clique aqui: [SolouChuva App](https://app.solouchuva.com.br/dashboard) (Ainda em produção, **não finalizado**)

> Obs.: Recuperação de senha e alteração de avatar não estão funcionais no [SolouChuva App](https://app.solouchuva.com.br/dashboard), devido a limitação de recursos para manter a aplicação online.

## Preview

## Requisitos

- [IDE do Arduino](https://www.arduino.cc/)
- Ter os componentes abaixo

**Componentes:**
* ESP8266 NodeMcu ESP-12 (Amica)
* Protobord 1660 Pontos

**Sensores:**
- [x] Sensor DHT22 (Temperatura e umidade do ar)
- [x] Sensor BMP280 (Temperatura, pressão e altitude)
- [x] Sensor UVM-30A (Luz ultravioleta)
- [x] Sensor BH1750 (Luminosidade)
- [x] Sensor de Umidade do Solo Capacitivo
- [x] Sensor Hall US1881/U18 (Para montar o Pluviometro e o Anemômetro)
- [x] Sensor de dector de chuva (Para montar o sensor de molhamento foliar)

**Modulos:**
- [x] Pluviometro de bascula digital (Sensor Hall US1881/U18 - Foi construido seguindo os passos do pessoal da [Pluvi.On](https://pluvion.com.br/) neste [link](https://www.instructables.com/id/Arduino-Modules-Rain-Sensor)) 
- [ ] Indicador de direção do vento (Futuramente)
- [x] Anemometro (Sensor Hall US1881/U18  - Foi adquirido pronto)
- [x] Painel solar
- [x] Bateria 3.7v 1200mah
- [x] Lipo-Po Rider v1.3 (Módulo gerenciador de carga de baterias lítio que permite o gerenciamento de energia produzida por meio de painéis solares)

**Siga os passos abaixo**

```bash
# Instale todos os componentes igual ao esquemático abaixo

```

### Esquemático

A figura abaixo ilustra a montagem em na protoboard dos componentes.

![picture](https://raw.githubusercontent.com/georgaugusto/solouchuva/master/public/iot-monitoramento-meteorologico_fzz.png)

**Clone o projeto e acesse a pasta**

```bash
$ git clone https://github.com/georgaugusto/solouchuva-hardware.git && cd solouchuva-hardware

# Caso precise instalar alguma biblioteca acesse a pasta 'libraries'

# Defina a rede Wi-fi e as variáveis do Firebase no arquivo 'iot-monitoramento-meteorologico.ino'

# Carregue o arquivo 'iot-monitoramento-meteorologico.ino' dentro do NodeMcu

# Muito bem, o projeto foi iniciado e tudo deve estar funcionando!
```

## Como contribuir

**Faça um fork deste repositório**

```bash
# Fork usando a linha de comando oficial do GitHub, caso você não tenha a CLI do GitHub, use o site para fazer isso.

$ gh repo fork georgaugusto/solouchuva-hardware
```

**Siga os passos abaixo**

```bash
# Clone sua fork
$ git clone your-fork-url && cd solouchuva-hardware

# Crie uma branch com suas alterações
$ git checkout -b my-feature

# Faça o commit com suas mudanças
$ git commit -m 'feat: My new feature'

# Envie o código para sua remote branch
$ git push origin my-feature
```

Depois que sua solicitação pull for merged, você pode excluir seu branch

## Licença

Este projeto está licenciado sob a Licença MIT - consulte a [LICENSE](LICENSE) arquivo para obter detalhes.

---

> A aplicação foi desenvolvida durante a graduação em Análise e Desenvolvimento de Sistemas na [FEMA](https://www.fema.edu.br) e em conjunto com o GoStack 11.0, organizado pela [Rocketseat](https://rocketseat.com.br/).

Feito com 💜 &nbsp;por Georg Augusto Schlegel 👋 &nbsp;[Mande um Aló](https://www.linkedin.com/in/georgaugusto/)
