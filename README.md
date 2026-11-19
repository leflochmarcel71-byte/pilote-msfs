# Bot pilote MSFS

Un bot qui pilote un avion dans Microsoft Flight Simulator 2020 / 2024 et se commande depuis
Telegram. Il tourne sur le PC du simulateur et parle au simulateur par SimConnect.

- **Au sol** : prépare l'avion (protections, moteur, volets, feux, frein de parking) et peut
  décoller seul une fois aligné sur la piste.
- **En vol** : montée, croisière (celle donnée par le contrôle aérien, sinon celle du dernier vol),
  descente vers l'aéroport d'arrivée lu dans le plan de vol, alertes carburant et relief.
- **Avions de ligne** (multimoteurs qui décrochent au-dessus de 90 kt) : automanette, 250 kt sous
  10 000 ft, Mach 0,78 au plus, volets rentrés cran par cran, aérofreins en descente, atterrissage
  par l'autoland de l'avion sur ILS (sinon l'avion t'est rendu à 3 NM).
- **À l'arrivée** : pause à 10 NM pour te rendre la main, ou, avec `atterrissage oui`, approche,
  atterrissage et arrêt sur la piste. `repetition` répète l'approche et l'arrondi sur une piste
  imaginaire en plein ciel, pour régler l'arrondi sans risquer l'avion.

> L'atterrissage automatique n'est validé que sur un avion simulé (`essai_atterrissage.py`),
> pas encore dans MSFS. Reste devant l'écran pendant les premiers essais.

## Installation

```bash
pip install -r requirements.txt
```

## Lancement

Double-clic sur `lancer_bot.bat` (ou `python pilote_auto.py`). Au premier lancement, le script
demande le jeton d'un bot créé avec @BotFather ; le premier compte Telegram qui écrit au bot en
devient le propriétaire. `demarrage_auto.bat` le fait ensuite démarrer sans fenêtre à chaque
ouverture de session Windows.

Envoie `aide` au bot pour la liste des commandes.

## Fichiers créés à côté du script (jamais publiés)

| Fichier | Contenu |
|---|---|
| `telegram.json` | jeton du bot et compte propriétaire : **secret** |
| `reglages.json` | dernière croisière, réglages, gouvernes retenues par avion |
| `journal_vol.txt` | journal des vols |
| `decollage_*.csv`, `atterrissage_*.csv` | mesures pour régler le bot |
| `aeroports.csv`, `pistes.csv` | listes OurAirports (domaine public), téléchargées au premier besoin |

## Essais sans le simulateur

```bash
python pilote_auto.py --test
```

```bash
python essai_atterrissage.py
```
