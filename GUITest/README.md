## Kode til at starte op for en simpel GUI (som burde virke og opdatere dynamisk)

### TODO:

- [ ] Ændre linje 73 til at være den rigtige sti til robot programmet (tror det skal være en fil (evt. en BASH fil))
- [ ] Gøre så linje 71 - 76 IKKE er kommentare længere. Dette er linjerne til at starte op for robotprogrammet.
- [ ] Lave tests med robot programmet kørende ved siden af.
- [ ] GAME OVER pop up ved enten ikke flere legal moves eller tid er gået. Skal vise hvem der vandt.

### Nødvendige libraries:
```Bash
sudo apt update
sudo apt install libwxgtk3.2-dev
```
Eller
```
sudo apt update
sudo apt install libwxgtk3.2-gtk3-dev
```

#### Tjek om det er installeret korrekt ved:
```
wx-config --version
```
Skulle gerne vise (eller lignende):
```
3.2.4
```

### For at compile:
```
g++ main.cpp $(wx-config --cxxflags --libs) -o GUITest
```
