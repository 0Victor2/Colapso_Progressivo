# Colapso_Progressivo

Repositório destinado ao desenvolvimento do trabalho colaborativo para a disciplina **MAC034 - Métodos Computacionais Aplicados em Engenharia**.

---

## 📝 Sobre o Projeto

O colapso progressivo ocorre quando a falha de um elemento estrutural local (como a perda de um pilar por impacto, explosão ou falha de material) desencadeia uma reação em cadeia, levando à ruína de elementos vizinhos e, eventualmente, ao colapso total ou parcial de toda a estrutura.

Este projeto propõe uma abordagem automatizada para mitigar esse risco em estruturas reticuladas através de técnicas de otimização estrutural.

### Estratégia de Solução

O objetivo principal é encontrar quais vigas e pilares devem receber intervenções para garantir a estabilidade global do edifício caso ocorra a perda repentina de suporte.

*   **Automação e Análise:** Desenvolvimento de um script em Python integrado ao Abaqus para criar o modelo paramétrico, simular cenários de remoção de um ou dois pilares quaisquer e extrair os resultados de falha.
*   **Representação do Reforço:** A otimização atuará modificando as propriedades dos elementos selecionados de duas formas:
    1.  **Upgrade de Material:** Substituição do concreto convencional por Concreto de Alto Desempenho (HPC).
    2.  **Aumento Geométrico:** Incremento nas dimensões da seção transversal das barras (vigas/pilares).

