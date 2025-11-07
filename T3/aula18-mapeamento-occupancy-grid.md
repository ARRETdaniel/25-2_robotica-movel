

## Slide 2: Introdução


### Explicação

Este slide introduz o **mapeamento** como uma tarefa fundamental para robôs que operam com pouca ou nenhuma supervisão[cite: 10, 11]. Ele destaca que os mapas são essenciais para outras três aplicações principais da robótica móvel:

* **Localização** (saber onde o robô está) [cite: 13]
* **Planejamento de caminhos** (decidir como chegar a um local) [cite: 14]
* **Tomada de decisão** (escolher ações com base no ambiente) [cite: 15]

---

## Slide 3: Mapeamento - Problemas


### Explicação

Este slide detalha os principais problemas enfrentados no mapeamento robótico:

* **Incertezas** [cite: 21]: O robô precisa lidar com dados de sensores que são "ruidosos" (imperfeitos) e com erros em sua própria estimativa de localização, que vêm da sua atuação (movimento)[cite: 22, 23].
* **Ambiguidade** [cite: 24]: É difícil para o robô estabelecer correspondência entre diferentes leituras de sensor, especialmente no problema de "Loop-Closure", que é a capacidade de reconhecer um local que já foi visitado anteriormente[cite: 25, 26].

A solução proposta para lidar com essas questões é utilizar uma **representação baseada em probabilidades**[cite: 27].

---

## Slide 4: Occupancy Grid


### Explicação

O slide introduz o **Occupancy Grid** (Grade de Ocupação)[cite: 31].
* É um **algoritmo probabilístico** [cite: 32], proposto por Moravec e Elfes em 1985[cite: 33].
* Baseia-se em algumas considerações principais:
    * A **localização do robô é considerada conhecida**[cite: 35].
    * O **ambiente é estático** (não muda)[cite: 36].
    * O mapa é dividido em uma **representação em grid** (grade)[cite: 37].
    * Este formato facilita a **integração de diferentes sensores**[cite: 38].

---

## Slide 5: Occupancy Grid (Definição)


### Explicação

Este slide define o modelo matemático do Occupancy Grid:

* Cada célula $i$ do grid é uma **variável aleatória binária** que modela a ocupação daquela posição[cite: 43].
    * $p(m_i) = 0$ significa que a célula está **vazia**[cite: 43].
    * $p(m_i) = 1$ significa que a célula está **ocupada**[cite: 44].
* A notação $p(m_i)$ é usada para representar a probabilidade da célula estar ocupada[cite: 45].
* Inicialmente, todas as células começam com $p(m_i) = 0,5$, que significa **desconhecido**[cite: 46].
* Uma suposição crucial é que as **probabilidades nas células são independentes** umas das outras[cite: 47].
* O mapa completo $m$ é a união de todas as células, e sua probabilidade total é o produto ( $\prod$ ) da probabilidade de cada célula individual: $p(m) = \prod_i p(m_i)$[cite: 48].

---

## Slide 6: Occupancy Grid (Visualização Inicial)


### Explicação

Esta imagem [Image from source 52] mostra a representação visual de um Occupancy Grid. O slide explica que **tons de cinza são usados para facilitar a visualização** das probabilidades[cite: 53].

Esta grade cinza uniforme representa o **estado inicial do mapa**, onde todas as células têm probabilidade 0.5, ou seja, são **desconhecidas** (como definido no slide anterior [cite: 46]).

---

## Slide 7: Occupancy Grid (Robô e Sensor)


### Explicação

Esta imagem [Image from source 57] mostra a grade com dois novos elementos:
1.  Um **robô**, representado pelo pequeno quadrado azul.
2.  O **campo de percepção** do sensor do robô (como um sonar ou laser), representado pela área vermelha em forma de leque.

O texto reforça uma das principais premissas do algoritmo: assume-se que a **posição do robô no ambiente é conhecida**[cite: 58, 59].

---

## Slide 8: Occupancy Grid (Atualização de Células)


### Explicação

Esta imagem [Image from source 63] ilustra como o mapa é atualizado após uma leitura do sensor (a área vermelha). O texto explica que, "considerando a incerteza associada à percepção, as células são preenchidas com as probabilidades de estarem ocupadas"[cite: 66].

* As células que ficam **brancas** [Image from source 63] estão entre o robô e a detecção do obstáculo. Elas recebem uma alta probabilidade de estarem **livres** ($p(m_i) \approx 0$).
* As células que ficam **escuras/pretas** [Image from source 63] estão na ponta do feixe do sensor, onde o obstáculo foi detectado. Elas recebem uma alta probabilidade de estarem **ocupadas** ($p(m_i) \approx 1$).
* As células cinzas fora do cone permanecem **desconhecidas** ($p(m_i) = 0.5$).

---

## Slide 9: Occupancy Grid (Interpretação das Células)


### Explicação

Esta imagem [Image from sources 68, 75] consolida a representação visual do mapa. Ela mostra setas que explicam o significado de cada cor:

* Uma seta aponta para uma célula **preta**, indicando que sua probabilidade de ocupação é $p(m_i) = 1$ (Ocupado)[cite: 75].
* Uma seta aponta para uma célula **branca**, indicando que sua probabilidade de ocupação é $p(m_j) = 0$ (Livre)[cite: 75].
* Uma seta aponta para uma célula **cinza** (desconhecida) [cite: 73], ligando-a a um histograma[cite: 74]. Este histograma mostra probabilidades iguais para "0" (livre) e "1" (ocupado), representando visualmente a probabilidade inicial de 0.5 (Desconhecido).

O texto na imagem reforça a suposição principal: **"Assumimos que não existe correlação entre as células!"**[cite: 69].

---

## Slide 10: Occupancy Grid (O Problema Matemático)


### Explicação

Este slide define o objetivo formal do mapeamento: determinar a **probabilidade a posteriori** do mapa $m$, dadas todas as observações dos sensores $z_{1:t}$ e todo o caminho percorrido pelo robô $x_{1:t}$[cite: 78, 79]. Isso é expresso como $p(m|z_{1:t}, x_{1:t})$.

* **O Problema:** Calcular isso diretamente é **intratável**. Um grid simples de $100 \times 100$ células teria $2^{10000}$ mapas binários possíveis[cite: 79].
* **A Solução (Aproximação):** O problema é simplificado. Em vez de calcular a probabilidade do mapa inteiro, o algoritmo calcula a **probabilidade marginal** para cada célula $i$ individualmente e depois as multiplica. Isso é possível por causa da suposição de independência das células[cite: 81]:
    $p(m|z_{1:t}, x_{1:t}) \cong \prod_i p(m_i|z_{1:t}, x_{1:t})$


## Slide 11: Occupancy Grid (Modelo Gráfico)



### Explicação

Esta imagem [Image from sources 85-94] mostra um **modelo gráfico** que representa as dependências entre as variáveis do problema.

* **$m$ (Mapa)**: É o mapa, que é estático. [cite_start]Ele influencia *todas* as leituras do sensor ($z_t, z_{t-1}, ...$)[cite: 89, 90, 91, 94].
* [cite_start]**$x$ (Estado do Robô)**[cite: 85, 87, 88]: A pose (localização e orientação) do robô ao longo do tempo. [cite_start]O estado atual $x_t$ depende apenas do estado anterior $x_{t-1}$ (isso é uma premissa de Markov)[cite: 86, 87, 88].
* [cite_start]**$z$ (Leitura do Sensor)** [cite: 85, 90, 92][cite_start]: A medição do sensor no tempo $t$ ($z_t$) depende apenas do **mapa $m$** e do **estado atual $x_t$**[cite: 87, 90, 94].

[cite_start]O slide reforça a fórmula de aproximação principal: a probabilidade do mapa inteiro é o produto das probabilidades de cada célula individual[cite: 95].

---

## Slide 12: Occupancy Grid (Cálculo)



### Explicação

Este slide faz a transição do "o quê" para o "como". [cite_start]A pergunta-chave é[cite: 100]:
> "Mas como calcular $p(m_{i}|z_{1:t},x_{1:t})$?"

Ou seja, como calcular a probabilidade de uma única célula $m_i$ dadas todas as medições e poses até agora?

[cite_start]A resposta é usar o **Teorema de Bayes Condicional** [cite: 101][cite_start], e a forma geral do teorema é apresentada[cite: 102].

---

## Slide 13: Occupancy Grid (Aplicação de Bayes)



### Explicação

[cite_start]Este slide aplica o Teorema de Bayes ao nosso problema[cite: 108]. [cite_start]Em seguida, ele usa **suposições de Markov** (independências) para simplificar drasticamente a equação[cite: 111, 114].

1.  [cite_start]**Simplificação da Medição**: A medição atual $z_t$ só depende do estado atual $x_t$ e da célula $m_i$[cite: 109]. [cite_start]Isso simplifica o primeiro termo no numerador[cite: 111].
2.  [cite_start]**Simplificação do Estado**: O estado atual $x_t$ (sem a medição $z_t$) não nos dá nenhuma nova informação sobre a célula $m_i$[cite: 110]. [cite_start]Isso simplifica o segundo termo no numerador[cite: 111].

[cite_start]O resultado [cite: 111] é uma **fórmula recursiva**. A probabilidade atual ($...|z_{1:t}, x_{1:t}$) depende da probabilidade anterior ($...|z_{1:t-1}, x_{1:t-1}$) e de um novo termo de medição ($p(z_t | m_i, x_t)$).

---

## Slide 14: Occupancy Grid (Continuação da Derivação)



### Explicação

[cite_start]Este slide continua a manipulação algébrica da fórmula do slide anterior[cite: 121]. [cite_start]Ele aplica o Teorema de Bayes novamente no termo $p(z_{t}|m_{i},x_{t})$ para isolar o termo $p(m_{i}|z_{t},x_{t})$[cite: 120, 121].

[cite_start]Ele também faz mais uma simplificação [cite: 123][cite_start], assumindo que a probabilidade da célula $m_i$ não depende do estado atual $x_t$ sozinho (sem a medição)[cite: 122]. O objetivo é chegar a uma equação que separe:
1.  A crença anterior sobre a célula ($p(m_i|z_{1:t-1}, x_{1:t-1})$).
2.  A informação da nova medição ($p(m_i|z_t, x_t)$).
3.  A crença inicial (prior) sobre a célula ($p(m_i)$).

---

## Slide 15: Occupancy Grid (Prior, Modelo, Posterior)

[Image contrasting the grid "Prior" (before reading) and "Posterior" (after reading)]

### Explicação

Esta imagem [Image from sources 129, 134] é uma excelente explicação visual dos termos da equação:

* [cite_start]**Prior $p(m_i | x_t)$** [cite: 130][cite_start]: Representa a crença *antes* de incorporar a medição atual[cite: 129]. [cite_start]Na imagem, a grade está toda cinza (desconhecida)[cite: 129].
* [cite_start]**Modelo de Observação $p(z_t | m_i, x_t)$**[cite: 131, 132]: É o modelo do sensor.
* [cite_start]**Posterior $p(m_i | z_t, x_t)$** [cite: 135][cite_start]: Representa a crença *após* incorporar *apenas* a medição atual $z_t$[cite: 134, 135]. [cite_start]A imagem mostra que esta medição (o raio vermelho) torna as células ao longo do raio **brancas (livres)** e a célula no final do raio **preta (ocupada)**[cite: 134]. Este termo é frequentemente chamado de **Modelo Inverso do Sensor**.

---

## Slide 16: Occupancy Grid (Problema com Probabilidades)



### Explicação

[cite_start]Este slide aponta que a manipulação direta de probabilidades é difícil[cite: 138, 139]. Multiplicar muitas probabilidades pequenas (números entre 0 e 1) pode levar a problemas numéricos (instabilidade ou *underflow*).

[cite_start]A solução proposta é usar a **razão das probabilidades** (chamada de "Odds" ou "Chance")[cite: 140]. [cite_start]Para isso, o slide apresenta a equação para a probabilidade de "não-ocupado" ($p(\neg m_i | ...)$), que é o complementar da equação para "ocupado"[cite: 141].

---

## Slide 17: Occupancy Grid (Definição de Odds)



### Explicação

[cite_start]Este slide define formalmente o que é **Odds**[cite: 146]:
* [cite_start]É a razão entre a probabilidade de um evento A acontecer e a probabilidade de ele não acontecer: $o(A) = p(A) / p(\neg A)$[cite: 149].
* [cite_start]O domínio das odds é $[0, +\infty)$[cite: 147].
* [cite_start]O slide também mostra a fórmula para recuperar a probabilidade $p(A)$ a partir das odds $o(A)$[cite: 150].

---

## Slide 18: Occupancy Grid (Derivação com Odds)



### Explicação

Este slide mostra a "mágica" da simplificação. [cite_start]Ele divide a equação de probabilidade para "ocupado" (do slide 14) [cite: 156] [cite_start]pela equação para "livre" (do slide 16)[cite: 159].

[cite_start]Ao fazer essa divisão, todos os termos complicados do denominador (os $p(z_t|...)$) se cancelam[cite: 160].

[cite_start]O resultado é uma equação **multiplicativa** muito mais simples[cite: 161]. Ela mostra que as "odds atuais" são simplesmente as "odds da medição" vezes as "odds anteriores" vezes um termo de correção da "prior inicial".

---

## Slide 19: Mapeamento (Log-Odds)



### Explicação

[cite_start]Este slide introduz a otimização final: **Log-Odds**[cite: 165].
* [cite_start]É simplesmente o logaritmo das odds: $l(A) = log(o(A))$[cite: 169].
* **Por que usar?**
    1.  [cite_start]**Eficiência/Estabilidade**[cite: 167]: Transforma a equação *multiplicativa* (do slide 18) em uma equação *aditiva* (pois $log(a \times b) = log(a) + log(b)$). Adição é computacionalmente mais rápida e estável que multiplicação.
    2.  [cite_start]**Evita Instabilidade Numérica**[cite: 168]: Lidar com probabilidades muito próximas de 0 ou 1 é numericamente instável. [cite_start]Log-odds mapeia o domínio $[0, \infty)$ para $(-\infty, +\infty)$[cite: 166], o que é muito mais estável.
    3.  [cite_start]Evita problemas de truncamento/arredondamento[cite: 168].

---

## Slide 20: Occupancy Grid (Equação Log-Odds)



### Explicação

[cite_start]Este slide começa a aplicar a transformação de log-odds à equação multiplicativa do slide 18[cite: 175].

[cite_start]Ele define a notação $l_{t,i}$ como sendo o valor de log-odds para a célula $i$ no tempo $t$ (ou seja, após $t$ medições)[cite: 176]. Esta é a forma final como a informação de cada célula será armazenada no mapa.



## Slide 21: Occupancy Grid (Equação Log-Odds Aditiva)

### Explicação

Este slide apresenta a **equação de atualização final** na forma de **log-odds**[cite: 175, 176]. O grande benefício é que a operação, que antes era uma multiplicação de probabilidades, torna-se uma **simples adição**[cite: 181].

A regra de atualização [cite: 182] é:
$l_{t,i} = l_{t-1,i} + inverse\_sense\_model(m_{i},x_{t},z_{t}) - l_{0}$ [cite: 183, 184]

Isso significa:

  * O **novo valor log-odds** da célula ($l_{t,i}$)
  * É igual ao **valor log-odds anterior** ($l_{t-1,i}$)
  * **Mais** a informação da **nova leitura do sensor** (o "modelo inverso do sensor")
  * **Menos** o valor log-odds **inicial** (o "prior", que é $log(0,5/0,5) = 0$, então esse termo $l_0$ geralmente é zero).

-----

## Slide 22: Occupancy Grid (Recuperando a Probabilidade)

### Explicação

Este slide mostra como fazer o caminho inverso: **converter o valor log-odds** (que é o que o mapa armazena) de volta para uma **probabilidade** (um número entre 0 e 1), que é mais fácil de visualizar e usar para tomada de decisão[cite: 189].

A fórmula para recuperar a probabilidade $p$ a partir do log-odds $l_{t,i}$ é[cite: 191]:

$p(m_{i}|z_{1:t},x_{1:t}) = 1 - \frac{1}{1 + exp(l_{t,i})}$

Esta é uma forma da função sigmóide (ou logística).

-----

## Slide 23: Occupancy Grid (Algoritmo Principal)

### Explicação

Este slide apresenta o **pseudocódigo do algoritmo principal** de mapeamento[cite: 198], que é chamado a cada nova leitura de sensor $z_t$.

O algoritmo itera por *todas as células* $m_i$ do grid[cite: 198]. Para cada célula, ele verifica:

1.  A célula $m_i$ está dentro do **campo de percepção** da leitura atual $z_t$? [cite: 199]
2.  **Se SIM:** A célula é atualizada usando a regra de adição log-odds que vimos no slide 21[cite: 200].
3.  **Se NÃO:** A célula está fora do alcance do sensor, então seu valor permanece o mesmo de antes ($l_{t,i} = l_{t-1,i}$)[cite: 206, 214].

Finalmente, o algoritmo retorna o grid atualizado $\{l_{t,i}\}$[cite: 213].

### Pseudocódigo para Implementação

```
Algorithm occupancy_grid_mapping( {l_t-1,i}, xt, zt):
  1:  for all cells m_i do
  2:    if m_i in perceptual field of z_t then
  3:      l_t,i = l_t-1,i + inverse_sensor_model(m_i, x_t, z_t) - l_0
  4:    else
  5:      l_t,i = l_t-1,i
  6:    endif
  7:  endfor
  8:  return {l_t,i}
```

*(Baseado nas linhas 198-214 do slide)*

-----

## Slide 24: Occupancy Grid (Resultado Visual)

### Explicação

Esta imagem [Image from source 217-241] é puramente conceitual e mostra o **resultado do algoritmo** ao longo do tempo.

  * Cada um dos **pequenos mapas** representa a informação de uma *única* varredura do sensor (o resultado da função $inverse\_sensor\_model$). Neles, vemos uma pequena área livre (branca/cinza claro) e uma fina camada de obstáculos (preto).
  * Os **sinais de mais (+)** [cite: 220-231, 234-237, 239] simbolizam a **soma** (adição de log-odds) que o algoritmo faz.
  * A **imagem final à direita** é o resultado da soma de todas essas leituras individuais, formando um mapa coerente do ambiente.

-----

## Slide 25: Occupancy Grid (Modelo Inverso do Sensor)

### Explicação

Este slide [Image from source 246-258] é a **chave** para entender a função $inverse\_sensor\_model$. Ele explica o que fazer com cada célula ao longo do raio de *um* feixe de sensor que detectou um obstáculo a uma certa distância (a "Leitura do sensor" [cite: 256]).

O gráfico mostra a probabilidade de ocupação (eixo Y) versus a distância da célula ao sensor (eixo X):

1.  **Células entre o robô e a leitura ($< z_{t,n} - r/2$):** [cite: 255] Essas células foram atravessadas pelo feixe do sensor e *não* detectaram nada. Portanto, elas "provavelmente estão livres" ($p_{free}$)[cite: 253, 254]. Elas recebem um valor log-odds **negativo** (livre).
2.  **Células na distância da leitura ($\approx z_{t,n} \pm r/2$):** [cite: 252] Estas são as células onde o obstáculo foi de fato detectado. Elas "provavelmente estão ocupadas" ($p_{occ}$)[cite: 248, 249]. Elas recebem um valor log-odds **positivo** (ocupado).
3.  **Células além da leitura ($> z_{t,n} + r/2$):** O feixe foi bloqueado pelo obstáculo, então "Não temos informações suficientes" sobre o que há *atrás* dele[cite: 250]. A probabilidade permanece a inicial ($p_{prior}$)[cite: 251], e o valor log-odds é **zero** (desconhecido).

-----

## Slide 26: Occupancy Grid (Pseudocódigo do Modelo Inverso)

### Explicação

Este slide fornece o **pseudocódigo para a função $inverse\_range\_sensor\_model$**[cite: 276], que implementa a lógica do slide anterior. O algoritmo faz o seguinte para *cada célula* $i$:

1.  Calcula a distância $r$ [cite: 268] e o ângulo $\phi$ [cite: 278] da célula $i$ em relação à pose do robô $x_t$.
2.  Encontra o feixe de sensor $k$ mais próximo daquele ângulo $\phi$[cite: 279].
3.  **Verifica se está fora do campo de visão:** Se a distância $r$ da célula for *maior* que a leitura do sensor $z_t^k$ (mais uma tolerância $\alpha/2$) OU se o ângulo $\phi$ da célula estiver fora do cone do feixe (largura $\beta/2$), então essa célula não é afetada. Retorna $l_0$ (desconhecido/sem mudança).
4.  **Verifica se está ocupada:** Se a leitura do sensor $z_t^k$ não for o alcance máximo E a distância $r$ da célula for *igual* à leitura $z_t^k$ (dentro da tolerância $\alpha/2$), a célula é considerada **ocupada**. Retorna $l_{occ}$ (um valor log-odds positivo)[cite: 275, 281, 283].
      * **Nota de implementação:** A linha 8  `|r - z_max|` parece ser um erro de digitação no slide; logicamente, deveria ser `|r - z_t^k|` para comparar a distância da célula com a *leitura* do sensor, não com o alcance *máximo*.
5.  **Verifica se está livre:** Se a distância $r$ da célula for *menor ou igual* à leitura $z_t^k$, ela está no espaço livre entre o robô e o obstáculo. Retorna $l_{free}$ (um valor log-odds negativo).

### Pseudocódigo para Implementação

```
Algorithm inverse_range_sensor_model l(i, xt, zt):
  1:  Let xi, yi be the center-of-mass of mi
  2:  r = sqrt((xi - x)^2 + (yi - y)^2)
  3:  phi = atan2(yi - y, xi - x) - theta
  4:  k = argmin_j |phi - theta_j,sens|

  5:  if r > min(z_max, z_t^k + alpha/2) or |phi - theta_k,sens| > beta/2 then
  6:    return l0
  7:  endif

  8:  if z_t^k < z_max and |r - z_t^k| < alpha/2 then  // Corrigido z_max para z_t^k com base no contexto
  9:    return l_occ
  10: endif

  11: if r <= z_t^k then
  12:   return l_free
  13: endif
```

*(Baseado nas linhas 268-289 do slide, com a correção de implementação anotada)*

-----
