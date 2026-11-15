"""
Bot pilote pour Microsoft Flight Simulator 2020/2024, commande depuis Telegram.

Le bot tourne en permanence sur le PC. Depuis Telegram tu lui envoies par exemple
"vol" tout court (avions a pilote automatique, pas les helicopteres) : il prend
l'altitude de croisiere que le controle aerien donne
apres le decollage (a defaut, celle de ton dernier vol), lit l'aeroport d'arrivee dans
le plan de vol du GPS et te rend l'avion 500 m au-dessus de ce terrain. Tu peux aussi
tout donner toi-meme : "vol 9000", "vol 9000 LFMD" ou "vol 9000 3300". Ensuite tu
decolles, il prend l'avion a 1000 ft sol, monte, tient la croisiere, descend et
met le simulateur en pause avant l'arrivee. Tu reprends la main en coupant le
pilote automatique. Avec "atterrissage oui", il fait l'approche, se pose et
s'arrete sur la piste ; "repetition" s'entraine sur une piste imaginaire en plein ciel.

Si l'avion est au parking quand tu envoies "vol ...", le bot le prepare tout seul :
protections enlevees (si l'avion le permet), moteur demarre, volets de decollage,
feux, frein de parking desserre. Tu roules jusqu'a la piste et, une fois aligne,
"decollage" fait decoller l'avion a ta place.

Installation (une fois) :   pip install SimConnect
Lancement :                 double-clic sur lancer_bot.bat   (ou : python pilote_auto.py)
Premier lancement : le script demande le jeton du bot cree avec @BotFather, puis
le premier compte Telegram qui lui ecrit devient son proprietaire.
Essai sans simulateur ni Telegram :   python pilote_auto.py --test
"""
import argparse
import csv
import json
import math
import os
import queue
import re
import shutil
import socket
import sys
import threading
import time
import urllib.request

NM = 1852.0  # metres


# --------------------------------------------------------------------------
# Acces au simulateur
# --------------------------------------------------------------------------
class SimReel:
    def __init__(self):
        from SimConnect import SimConnect, AircraftRequests, AircraftEvents
        self.sm = SimConnect()
        self.aq = AircraftRequests(self.sm, _time=500)
        self.aq_vite = AircraftRequests(self.sm, _time=40)   # lectures rapides, pour le decollage
        self.ae = AircraftEvents(self.sm)
        self._libres = {}

    def lire(self, nom):
        try:
            return self.aq.get(nom)
        except Exception:
            return None

    def lire_vite(self, nom):
        try:
            return self.aq_vite.get(nom)
        except Exception:
            return None

    def variable(self, nom, unite="Bool", valeur=None):
        """Lit (ou regle, si 'valeur' est donnee, puis relit) une variable du simulateur
        que la bibliotheque ne connait pas. Renvoie None si le simulateur ne repond pas."""
        try:
            from SimConnect.RequestList import Request
            r = self._libres.get(nom)
            if r is None:
                r = self._libres[nom] = Request((nom.encode(), unite.encode()), self.sm, _time=0, _settable=True)
            if valeur is not None:
                r.value = float(valeur)
                time.sleep(0.5)
            r.outData = None          # force une vraie relecture
            return r.value
        except Exception:
            return None

    def envoyer(self, nom, valeur=None):
        try:
            ev = self.ae.find(nom)
            if ev is None:
                return False
            ev(int(valeur)) if valeur is not None else ev()
            return True
        except Exception:
            return False

    def attendre(self, s):
        time.sleep(s)

    def horloge(self):
        return time.monotonic()


class Tampon:
    """Enveloppe le simulateur : quand une lecture echoue (la bibliotheque renvoie
    parfois None), on reprend la derniere valeur connue si elle a moins de 10 s."""

    def __init__(self, sim):
        self._sim, self._vus = sim, {}

    def lire(self, nom):
        v = self._sim.lire(nom)
        if v is not None:
            self._vus[nom] = (v, time.monotonic())
            return v
        ancien = self._vus.get(nom)
        return ancien[0] if ancien and time.monotonic() - ancien[1] < 10 else None

    def __getattr__(self, nom):
        return getattr(self._sim, nom)


class SimFictif:
    """Avion tres simplifie pour verifier la logique sans le simulateur."""
    fictif = True

    def __init__(self, distance_nm=120.0, decollage_auto=False, sens=(-1, -1)):
        # decollage_auto : personne ne pilote au sol, c'est au bot de faire decoller l'avion.
        # sens : sens (inconnu du bot) des commandes de palonnier et de profondeur.
        self.auto, self.sens = decollage_auto, sens
        self.cap = math.degrees(1.0)
        self.palonnier, self.profondeur = 0.0, 0.0       # +1 = a droite / a cabrer
        self.frein_park, self.volets, self.train = True, 0, True
        self.caches = [1, 1, 1]
        self.moteur, self.demarrage = True, 0
        self.sourd = set()                        # commandes de train que cet avion ignore
        self.controle = lambda t, alt: 0          # altitude assignee par le controle (aucune par defaut)
        self.conception = {"DESIGN SPEED VS1": 62.0, "DESIGN TAKEOFF SPEED": 88.0, "DESIGN SPEED CLIMB": 140.0}
        self.freins = 0
        self.t = 0.0
        self.alt, self.sol = 0.0, 0.0
        self.ias, self.vs = 0.0, 0.0
        self.dist = distance_nm
        self.ap, self.mode_vs = False, False
        self.alt_sel, self.vs_sel = 0.0, 0.0
        self.gaz = 0.0 if decollage_auto else 0.9     # au ralenti tant que le bot n'a pas mis les gaz
        self.hdg, self.bug = False, 0
        self.pause = 0

    def lire(self, nom):
        return {
            "SIM_ON_GROUND": self.alt <= self.sol + 1,
            "PLANE_ALT_ABOVE_GROUND": self.alt - self.sol,
            "PLANE_ALTITUDE": self.alt - 400,      # altitude vraie, differente de l'altimetre
            "INDICATED_ALTITUDE": self.alt,
            "AIRSPEED_INDICATED": self.ias,
            "VERTICAL_SPEED": self.vs,
            "AUTOPILOT_MASTER": self.ap,
            "GPS_IS_ACTIVE_FLIGHT_PLAN": True,
            "GPS_DRIVES_NAV1": True,
            "GPS_ETE": self.dist * NM / max(self.ias * 0.5144, 1),
            "GPS_GROUND_SPEED": self.ias * 0.5144,
            "PLANE_BANK_DEGREES": 0.0,
            "PLANE_HEADING_DEGREES_MAGNETIC": math.radians(self.cap),
            "PLANE_HEADING_DEGREES_TRUE": math.radians(self.cap),
            "RUDDER_POSITION": self.palonnier,
            "ELEVATOR_POSITION": self.profondeur,
            "BRAKE_PARKING_POSITION": self.frein_park,
            "FLAPS_HANDLE_INDEX": self.volets,
            "GENERAL_ENG_COMBUSTION:1": self.moteur,
            "NUMBER_OF_ENGINES": 1, "AUTOPILOT_AVAILABLE": True, "CATEGORY": b"Airplane",
            "GEAR_HANDLE_POSITION": self.train, "IS_GEAR_RETRACTABLE": True,
            "AUTOPILOT_VERTICAL_HOLD_VAR": self.vs_sel, "AUTOPILOT_VERTICAL_HOLD": self.mode_vs,
            "AUTOPILOT_ALTITUDE_LOCK_VAR": self.alt_sel,
            "GROUND_VELOCITY": self.ias,
            "PLANE_LATITUDE": math.radians(43.542 + self.dist / 60),    # plein nord de l'arrivee fictive
            "PLANE_LONGITUDE": math.radians(6.95347),
            "GPS_WP_BEARING": 1.0 + self.t / 5000,
            "GPS_FLIGHT_PLAN_WP_COUNT": 3, "GPS_FLIGHT_PLAN_WP_INDEX": 2 if self.dist < 100 else 1,
            "GPS_WP_NEXT_ID": b"LFMD" if self.dist < 100 else b"NIZ",
            "GPS_WP_NEXT_LAT": 43.542, "GPS_WP_NEXT_LON": 6.95347,
            "GPS_APPROACH_AIRPORT_ID": b"",
            "AUTOPILOT_HEADING_LOCK": self.hdg,
            "AUTOPILOT_HEADING_LOCK_DIR": self.bug,
            "GENERAL_ENG_THROTTLE_LEVER_POSITION:1": self.gaz * 100,
            "AUTOPILOT_THROTTLE_ARM": False,
            "STALL_WARNING": False,
            "OVERSPEED_WARNING": self.ias > 180,
        }.get(nom)

    lire_vite = lire

    def variable(self, nom, unite="Bool", valeur=None):
        if nom == "ATC ASSIGNED ALTITUDE":
            return float(self.controle(self.t, self.alt))
        if nom in self.conception:
            return self.conception[nom]
        if nom == "GEAR HANDLE POSITION":
            if valeur is not None and nom not in self.sourd:
                self.train = bool(valeur)
            return float(self.train)
        if nom == "FLAPS HANDLE INDEX":
            if valeur is not None and nom not in self.sourd:
                self.volets = int(valeur)
            return float(self.volets)
        if not nom.startswith("COVER ON:"):
            return None
        i = int(nom.split(":")[1])
        if i >= len(self.caches):
            return 0.0
        if valeur is not None:
            self.caches[i] = valeur
        return float(self.caches[i])

    def envoyer(self, nom, valeur=None):
        if nom in self.sourd:
            return True
        if nom == "AUTOPILOT_ON":
            self.ap = True
        elif nom == "FLAPS_SET":
            self.volets = round(valeur / 16383 * 2)
        elif nom == "FLAPS_DECR":
            self.volets = max(self.volets - 1, 0)
        elif nom == "RUDDER_SET":
            self.palonnier = self.sens[0] * valeur / 16383
        elif nom == "RUDDER_RIGHT":
            self.palonnier = min(self.palonnier + 0.05, 1)
        elif nom == "ELEVATOR_SET":
            self.profondeur = self.sens[1] * valeur / 16383
        elif nom == "ELEV_UP":
            self.profondeur = min(self.profondeur + 0.05, 1)
        elif nom == "PARKING_BRAKES":
            self.frein_park = not self.frein_park
        elif nom == "FLAPS_1":
            self.volets = 1
        elif nom == "FLAPS_UP":
            self.volets = 0
        elif nom == "GEAR_UP":
            self.train = False
        elif nom == "GEAR_SET":
            self.train = bool(valeur)
        elif nom == "GEAR_TOGGLE":
            self.train = not self.train
        elif nom == "BRAKES":
            self.freins = 3
        elif nom == "ENGINE_AUTO_START":
            self.demarrage = 20
        elif nom == "AP_ALT_VAR_SET_ENGLISH":
            self.alt_sel = valeur
        elif nom == "AP_VS_VAR_SET_ENGLISH":
            # Constate en vol reel : une valeur negative est prise pour une tres grande valeur positive.
            self.vs_sel = valeur if valeur >= 0 else 2000
        elif nom == "AP_VS_VAR_INC":
            self.vs_sel += 100
        elif nom == "AP_VS_VAR_DEC":
            self.vs_sel -= 100
        elif nom in ("AP_PANEL_VS_ON", "AP_VS_ON"):
            self.mode_vs = True
        elif nom == "THROTTLE_SET":
            self.gaz = valeur / 16383
        elif nom == "PAUSE_ON":
            self.pause = 5
        elif nom == "PAUSE_OFF":
            self.pause = 0
        elif nom == "HEADING_BUG_SET":
            self.bug = valeur
        elif nom == "AP_HDG_HOLD_ON":
            self.hdg = True
        return True

    def horloge(self):
        return self.t

    def attendre(self, s):
        if s < 1:
            self._pas(s)
        else:
            for _ in range(int(s)):
                self._pas(1)

    def _pas(self, dt):
        # Modele : decollage (manuel ou fait par le bot) puis l'AP suit VS / altitude.
        if True:
            self.t += dt
            if self.demarrage:
                self.demarrage = max(self.demarrage - dt, 0)
                self.moteur = not self.demarrage
            if self.pause:
                self.pause = max(self.pause - dt, 0)
                return
            if not self.ap and self.auto:
                au_sol = self.alt <= self.sol + 1
                if au_sol:
                    pousse = 0 if self.frein_park else 5 * self.gaz - 1
                    if self.freins:
                        pousse, self.freins = -8, self.freins - 1
                    self.ias = max(self.ias + pousse * dt, 0)
                    # vent de travers qui pousse le nez a gauche, palonnier efficace avec la vitesse
                    if self.ias > 1:
                        self.cap += (-1.5 + 25 * self.palonnier * min(self.ias, 60) / 60) * dt
                    self.vs = 3000 * self.profondeur if self.ias >= 85 and self.profondeur > 0.15 else 0.0
                else:
                    self.ias = min(self.ias + 1.5 * dt, 140)
                    self.vs += (6000 * self.profondeur - self.vs) * min(dt, 1)
                    self.cap += 4 * self.palonnier * dt
            elif not self.ap:
                self.ias = min(self.ias + 3 * dt, 110)
                self.vs = 900.0 if self.ias > 70 else 0.0
            else:
                ecart = self.alt_sel - self.alt
                if self.mode_vs and abs(ecart) > 50 and ecart * self.vs_sel > 0:
                    self.vs = self.vs_sel
                else:
                    self.vs = max(-500, min(500, ecart * 5))
                cible = 60 + 120 * self.gaz - self.vs / 40   # plus vite en descente
                self.ias += (cible - self.ias) * 0.05 * dt
            self.alt = max(self.alt + self.vs / 60 * dt, self.sol)
            self.dist = max(self.dist - self.ias / 3600 * dt, 0)


# --------------------------------------------------------------------------
# Telegram
# --------------------------------------------------------------------------
class TelegramMuet:
    """Utilise quand Telegram est desactive : ne fait rien."""

    def envoyer(self, texte):
        pass

    def position(self, lat, lon):
        pass

    def commandes(self):
        return []


class TelegramFictif(TelegramMuet):
    """Pour l'essai sans simulateur : affiche les envois et simule quelques messages recus."""

    def __init__(self, scenario=None):
        self.n = 0
        self.scenario = scenario or {3: ["vol 9000 3300"], 200: ["etat"], 900: ["pause"],
                                     903: ["reprendre"], 1500: ["bonjour"]}

    def envoyer(self, texte):
        print("      [TELEGRAM] " + texte.replace("\n", " | "), flush=True)

    def position(self, lat, lon):
        print(f"      [TELEGRAM] position {lat:.2f}, {lon:.2f}", flush=True)

    def commandes(self):
        self.n += 1
        return self.scenario.get(self.n, [])


class Telegram(TelegramMuet):
    """Bot Telegram : envois et reception dans des fils separes, pour ne jamais bloquer le vol."""

    def __init__(self, fichier):
        self.fichier = fichier
        self.debut = time.time()
        conf = {}
        if os.path.exists(fichier):
            try:
                conf = json.load(open(fichier, encoding="utf-8"))
            except Exception:
                conf = {}
        self.jeton, self.chat = conf.get("jeton"), conf.get("chat")
        self.recus, self.sortie = queue.Queue(), queue.Queue()

    def configurer(self):
        """Demande le jeton au premier lancement. Renvoie False si Telegram reste desactive."""
        while not self.jeton:
            print("\n--- Telegram (a faire une seule fois) ---\n"
                  "1. Dans Telegram, ouvre une discussion avec @BotFather et envoie /newbot.\n"
                  "2. Donne un nom au bot, puis un identifiant qui finit par 'bot'.\n"
                  "3. BotFather te donne un jeton (une longue suite de chiffres et de lettres).\n")
            saisie = input("Colle le jeton ici (ou appuie sur Entree pour voler sans Telegram) : ").strip()
            if not saisie:
                return False
            self.jeton = saisie
            if self._appel("getMe") is None:
                print("Ce jeton est refuse par Telegram. Verifie-le et recommence.")
                self.jeton = None
        moi = self._appel("getMe")
        if moi is None:
            print("Telegram injoignable (connexion ? jeton ?) : je vole sans Telegram.")
            return False
        self._sauver()
        if self.chat is None:
            print(f"\nOuvre Telegram, cherche @{moi.get('username')} et envoie-lui un message.\n"
                  "Le premier compte qui lui ecrit devient son proprietaire.\n")
        threading.Thread(target=self._boucle_envoi, daemon=True).start()
        threading.Thread(target=self._boucle_reception, daemon=True).start()
        return True

    def _sauver(self):
        json.dump({"jeton": self.jeton, "chat": self.chat}, open(self.fichier, "w", encoding="utf-8"))

    def _appel(self, methode, attente=15, **params):
        try:
            req = urllib.request.Request(
                f"https://api.telegram.org/bot{self.jeton}/{methode}",
                data=json.dumps(params).encode("utf-8"),
                headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=attente) as r:
                rep = json.load(r)
            return rep.get("result") if rep.get("ok") else None
        except Exception:
            return None

    def envoyer(self, texte):
        self.sortie.put(("sendMessage", {"text": texte}))

    def position(self, lat, lon):
        self.sortie.put(("sendLocation", {"latitude": lat, "longitude": lon}))

    def commandes(self):
        recues = []
        while not self.recus.empty():
            recues.append(self.recus.get())
        return recues

    def _boucle_envoi(self):
        while True:
            methode, params = self.sortie.get()
            while self.chat is None:          # pas encore de proprietaire : on attend
                time.sleep(1)
            for _ in range(3):                # trois essais si le reseau flanche
                if self._appel(methode, chat_id=self.chat, **params) is not None:
                    break
                time.sleep(5)

    def _boucle_reception(self):
        suivant = 0
        while True:
            maj = self._appel("getUpdates", attente=35, offset=suivant, timeout=25)
            if maj is None:
                time.sleep(5)
                continue
            for u in maj:
                suivant = u["update_id"] + 1
                m = u.get("message") or {}
                texte, chat = m.get("text"), (m.get("chat") or {}).get("id")
                if not texte or chat is None or m.get("date", 0) < self.debut - 5:
                    continue                   # vieux message envoye avant le lancement
                if self.chat is None:
                    self.chat = chat
                    self._sauver()
                    self.envoyer("Bot pilote connecte. Tu es mon proprietaire.\n" + AIDE)
                    continue
                if chat != self.chat:
                    continue                   # quelqu'un d'autre : ignore
                self.recus.put(texte)


AIDE = ("UN VOL, DANS L'ORDRE\n"
        "1. vol - avion a l'arret au parking : je le prepare (protections, moteur, volets, feux, "
        "frein de parking)\n"
        "2. Tu roules jusqu'a la piste et tu t'alignes\n"
        "3. decollage - je decolle, je rentre le train puis les volets, je monte, je tiens la "
        "croisiere et je descends (ou decolle toi-meme : je prends l'avion a 1000 ft sol)\n"
        "4. A l'arrivee, deux facons de finir :\n"
        "   - normal : a 10 NM, je mets en pause, tu coupes le pilote automatique et tu atterris\n"
        "   - avec 'atterrissage oui' : je fais l'approche, je me pose et je m'arrete sur la piste ; "
        "a toi ensuite pour la degager\n"
        "\n"
        "ATTERRISSAGE AUTOMATIQUE\n"
        "atterrissage oui / non - je me pose moi-meme, ou je te rends l'avion a 10 NM (retenu "
        "pour les vols suivants)\n"
        "piste 22 - piste voulue, jusqu'au debut de l'approche (sinon la plus face au vent)\n"
        "Il me faut : le code de l'aeroport d'arrivee (vol 9000 LFMD, le plan de vol ou "
        "arrivee LFMD), et une seule fois par avion, l'avion a l'arret au sol quand tu envoies 'vol' "
        "(je mesure ses gouvernes et je les retiens)\n"
        "Sans ses gouvernes : je fais l'approche et je te rends l'avion a 3 NM, en pause\n"
        "repetition - en montee ou en croisiere : approche et arrondi sur une piste imaginaire 2000 ft "
        "sous l'avion, puis je remonte et je reprends le vol (il faut 3500 ft de vide sous l'avion)\n"
        "L'approche commence a 18 NM ; si l'ILS de la piste est capte, le mode APR de l'avion le suit\n"
        "Pas stabilise a 500 ft (axe, plan, vitesse, train) : remise de gaz, et c'est a toi\n"
        "\n"
        "CE QUE JE TROUVE TOUT SEUL\n"
        "Croisiere : celle que le controle donne apres le decollage, sinon celle de ton dernier vol\n"
        "Arrivee : lue dans le plan de vol ; je te rends l'avion 500 m au-dessus du terrain\n"
        "Taux de montee : regle tout seul sur la vitesse de l'avion\n"
        "Vitesse de cabre au decollage : celle de l'avion charge\n"
        "Vol trop court pour monter puis descendre : je me mets en palier\n"
        "Aussi : altimetre recale, phares selon l'altitude, alertes carburant et relief\n"
        "\n"
        "POUR CHOISIR TOI-MEME (au premier vol, donne au moins la croisiere)\n"
        "vol 30000 - croisiere a 30000 ft\n"
        "vol 30000 LFMD - croisiere et aeroport d'arrivee\n"
        "vol 9000 3300 - croisiere, et altitude ou je te rends l'avion\n"
        "A ajouter a la fin : taux 1800 (taux de montee impose ; taux auto pour revenir au reglage "
        "automatique), cap 270 (cap garde tel quel)\n"
        "\n"
        "AU SOL\n"
        "prepare - refait la preparation de l'avion\n"
        "moteur - demarre seulement le moteur\n"
        "decollage 95 - decollage en cabrant a 95 kt au lieu de la vitesse de l'avion\n"
        "\n"
        "EN VOL\n"
        "etat - position, altitude, distance, temps restant et carburant\n"
        "suivi 10 - m'envoie l'etat toutes les 10 minutes (suivi 0 pour arreter)\n"
        "altitude 11000 - change la croisiere (je ne suis plus le controle)\n"
        "arrivee LFMD - donne ou change l'aeroport d'arrivee\n"
        "cap 270 - prend ce cap et le garde\n"
        "route - reprend le plan de vol\n"
        "descente - descend tout de suite\n"
        "\n"
        "A TOUT MOMENT\n"
        "pause / reprendre - pause du simulateur\n"
        "stop - j'interromps le decollage, ou je lache l'avion (sauf pendant l'arrondi, environ 30 s)\n"
        "aide - ce message")
TG = TelegramMuet()
_derniers = {}
JOURNAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "journal_vol.txt")
AEROPORTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aeroports.csv")
REGLAGES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reglages.json")
PISTES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pistes.csv")


def lire_reglages():
    """Croisiere et taux de montee du dernier vol, pour ne pas avoir a les retaper."""
    try:
        with open(REGLAGES, encoding="utf-8") as f:
            r = json.load(f)
        return r if isinstance(r, dict) else {}
    except Exception:
        return {}


def garder_reglages(**nouveaux):
    r = lire_reglages()
    r.update({k: v for k, v in nouveaux.items() if v})
    try:
        with open(REGLAGES, "w", encoding="utf-8") as f:
            json.dump(r, f)
    except Exception:
        pass


def prendre_croisiere_du_controle(sim, a):
    """Si on suit le controle et qu'il a assigne une altitude differente de la croisiere
    prevue, elle devient la croisiere. Renvoie True quand la croisiere vient de changer."""
    if not getattr(a, "suivre_controle", False):
        return False
    assignee = altitude_controle(sim)
    if not assignee or assignee == a.croisiere or (a.approche is not None and assignee <= a.approche + 500):
        return False
    a.croisiere = assignee
    dire(f"Le controle a donne {assignee} ft : c'est ma croisiere.", tel=True)
    return True


def fiche_avion(sim):
    """Ce que le simulateur dit de l'avion charge : vitesses de reference (en kt, None si
    inconnues), nombre de moteurs, presence d'un pilote automatique, categorie."""
    def vitesse(nom):
        v = num(sim.variable(nom, "Knots"))
        return v if 25 <= v <= 400 else None

    vs1 = vitesse("DESIGN SPEED VS1")
    vr = vitesse("DESIGN SPEED MIN ROTATION") or vitesse("DESIGN TAKEOFF SPEED")
    if vs1 and (vr is None or vr < 1.1 * vs1):
        vr = 1.2 * vs1                  # vitesse de cabre absente ou trop basse : 20 % au-dessus du decrochage
    return {"vs0": vitesse("DESIGN SPEED VS0"), "vs1": vs1, "vr": round(vr) if vr else None,
            "montee": vitesse("DESIGN SPEED CLIMB"),
            "moteurs": min(max(int(num(sim.lire("NUMBER_OF_ENGINES"), 1)), 1), 4),
            "ap": booleen(sim.lire("AUTOPILOT_AVAILABLE")), "categorie": texte_sim(sim.lire("CATEGORY"))}


def moteurs_en_marche(sim):
    """True si tous les moteurs tournent, False si l'un est arrete, None si illisible."""
    nb = min(max(int(num(sim.lire("NUMBER_OF_ENGINES"), 1)), 1), 4)
    etats = [booleen(frais(sim, f"GENERAL_ENG_COMBUSTION:{i}")) for i in range(1, nb + 1)]
    return False if False in etats else None if None in etats else True


def autonomie_h(sim):
    """Heures de vol restantes au debit de carburant actuel, ou None si illisible."""
    quantite = num(sim.lire("FUEL_TOTAL_QUANTITY"), None)
    nb = min(max(int(num(sim.lire("NUMBER_OF_ENGINES"), 1)), 1), 4)
    debit = sum(num(sim.lire(f"ENG_FUEL_FLOW_GPH:{i}")) for i in range(1, nb + 1))
    if quantite is None or quantite <= 0 or debit < 0.5:
        return None
    return quantite / debit


def altitude_controle(sim):
    """Altitude que le controle aerien a assignee a l'avion, en pieds arrondis a 100, ou
    None s'il n'en a pas donne (ou si le simulateur ne la transmet pas)."""
    v = num(sim.variable("ATC ASSIGNED ALTITUDE", "Feet"))
    return int(round(v / 100) * 100) if 1000 <= v <= 45000 else None
URL_AEROPORTS = "https://davidmegginson.github.io/ourairports-data/airports.csv"   # OurAirports, domaine public
HAUTEUR_REMISE_M = 500      # hauteur au-dessus du terrain d'arrivee ou le bot rend l'avion


URL_PISTES = "https://davidmegginson.github.io/ourairports-data/runways.csv"       # OurAirports, domaine public


def telecharger(url, chemin, quoi):
    """Telecharge un fichier de donnees s'il n'est pas deja a cote du script."""
    if os.path.exists(chemin):
        return
    TG.envoyer(f"Je telecharge {quoi}.")
    req = urllib.request.Request(url, headers={"User-Agent": "bot-pilote-msfs"})
    with urllib.request.urlopen(req, timeout=60) as r, open(chemin + ".tmp", "wb") as f:
        shutil.copyfileobj(r, f)
    os.replace(chemin + ".tmp", chemin)


_AEROPORTS = []
_COLONNES =("ident", "name", "latitude_deg", "longitude_deg", "elevation_ft", "icao_code", "gps_code", "local_code")


def liste_aeroports():
    """Liste OurAirports (telechargee une fois, puis gardee a cote du script), lue une
    seule fois puis gardee en memoire : la relire a chaque recherche prend du temps."""
    if _AEROPORTS:
        return _AEROPORTS
    telecharger(URL_AEROPORTS, AEROPORTS, "la liste des aeroports (une seule fois, environ 12 Mo)")
    with open(AEROPORTS, encoding="utf-8", newline="") as f:
        _AEROPORTS.extend({k: ligne.get(k) for k in _COLONNES} for ligne in csv.DictReader(f)
                          if ligne.get("type") not in ("closed", "heliport", "balloonport"))
    return _AEROPORTS


def chercher_aeroport(code=None, pres_de=None):
    """Cherche un aeroport par son code ou, avec pres_de=(lat, lon), le plus proche de ce
    point. Renvoie son code, son nom, son altitude en pieds (None si inconnue) et sa
    position, ou None s'il n'y en a pas. Leve une erreur si la liste n'a pas pu etre telechargee."""
    code = (code or "").upper()
    trouve, ecart = None, 3.0 if pres_de else None       # par position : a moins de 3 NM
    for ligne in liste_aeroports():
        if pres_de:
            try:
                d = distance_nm(pres_de[0], pres_de[1], float(ligne["latitude_deg"]), float(ligne["longitude_deg"]))
            except (KeyError, TypeError, ValueError):
                continue
            if d < ecart:
                trouve, ecart = ligne, d
        elif (ligne.get("ident") or "").upper() == code:
            trouve = ligne                      # le code exact passe avant les autres codes
            break
        elif trouve is None and code in ((ligne.get(k) or "").upper()
                                         for k in ("icao_code", "gps_code", "local_code")):
            trouve = ligne
    if trouve is None:
        return None
    try:
        alt = float(trouve.get("elevation_ft") or "")
    except ValueError:
        alt = None
    return {"code": (trouve.get("ident") or code).upper(), "nom": trouve.get("name") or code, "alt_ft": alt,
            "lat": float(trouve["latitude_deg"]), "lon": float(trouve["longitude_deg"])}


def distance_nm(lat1, lon1, lat2, lon2):
    """Distance en NM entre deux points donnes en degres."""
    la1, lo1, la2, lo2 = map(math.radians, (lat1, lon1, lat2, lon2))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 6371000 * math.asin(math.sqrt(h)) / NM


def angle(x):
    """Angle ramene entre -180 et +180 degres."""
    return (x + 180) % 360 - 180


def cap_vers(lat1, lon1, lat2, lon2):
    """Cap vrai, en degres, pour aller du premier point au second."""
    la1, la2, dl = math.radians(lat1), math.radians(lat2), math.radians(lon2 - lon1)
    return math.degrees(math.atan2(math.sin(dl) * math.cos(la2),
                                   math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dl))) % 360


def choisir_piste(code, vent_de=0.0, vent_kt=0.0, voulue=None):
    """Piste d'arrivee dans la liste OurAirports : celle demandee ('22', '04L'), sinon la plus
    face au vent (la plus longue par vent faible). Renvoie le seuil, l'axe vrai, l'altitude,
    la longueur, la largeur et le point vise (300 m apres le seuil), ou None."""
    telecharger(URL_PISTES, PISTES, "la liste des pistes (une seule fois, environ 4 Mo)")
    code, voulue = code.upper(), (voulue or "").upper().lstrip("0")
    meilleure, note_max = None, None
    with open(PISTES, encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            if ((r.get("airport_ident") or "").upper() != code or r.get("closed") == "1"
                    or "WATER" in (r.get("surface") or "").upper()):
                continue
            for bout, autre in (("le", "he"), ("he", "le")):
                try:
                    lat, lon = float(r[bout + "_latitude_deg"]), float(r[bout + "_longitude_deg"])
                    lat2, lon2 = float(r[autre + "_latitude_deg"]), float(r[autre + "_longitude_deg"])
                except (KeyError, TypeError, ValueError):
                    continue                        # piste sans coordonnees : inutilisable
                ident = (r.get(bout + "_ident") or "").upper()
                if voulue and ident.lstrip("0") != voulue:
                    continue
                cap = num(r.get(bout + "_heading_degT"), None)
                cap = cap_vers(lat, lon, lat2, lon2) if cap is None else cap % 360
                longueur = distance_nm(lat, lon, lat2, lon2) * NM
                face = vent_kt * math.cos(math.radians(vent_de - cap)) if vent_kt >= 5 else 0
                note = (round(face), longueur)
                if note_max is None or note > note_max:
                    decale = num(r.get(bout + "_displaced_threshold_ft")) * 0.3048
                    meilleure, note_max = {
                        "ident": ident, "lat": lat, "lon": lon, "cap": cap, "longueur_m": longueur,
                        "alt_ft": num(r.get(bout + "_elevation_ft"), None),
                        "largeur_m": num(r.get("width_ft"), 100) * 0.3048,
                        "visee_m": min(decale + 300, longueur / 3)}, note
    return meilleure


def sur_piste(piste, lat, lon):
    """Position par rapport au seuil, en metres : (avant, ecart). 'avant' est positif avant le
    seuil, cote approche ; 'ecart' est positif a droite de l'axe, en regardant vers la piste."""
    x = math.radians(lon - piste["lon"]) * 6371000 * math.cos(math.radians(piste["lat"]))
    y = math.radians(lat - piste["lat"]) * 6371000
    c = math.radians(piste["cap"])
    return -(x * math.sin(c) + y * math.cos(c)), x * math.cos(c) - y * math.sin(c)


def analyser(brut):
    """Renvoie le premier mot du message (sans accents) et les nombres qu'il contient."""
    t = brut.strip().lower().lstrip("/")
    for avec, sans in (("é", "e"), ("è", "e"), ("ê", "e"), ("à", "a")):
        t = t.replace(avec, sans)
    mots = t.split()
    return (mots[0] if mots else ""), [int(n) for n in re.findall(r"\d+", t)], t


def lire_vol(t):
    """Decode 'vol 9000 3300 [cap] [taux de montee]' ou 'vol 9000 LFMD [cap] [taux de montee]'.
    Renvoie un dictionnaire, ou None si mal ecrit. Avec un code d'aeroport, l'altitude de
    remise n'est pas donnee ('approche' vaut None) : elle sera tiree de l'altitude du terrain.
    Apres les altitudes, un nombre jusqu'a 360 est un cap, un nombre plus grand un taux de montee."""
    m = re.search(r"cap\s*(\d+)", t)
    cap = int(m.group(1)) if m else None
    m = re.search(r"taux\s*(\d+)", t)
    taux_ecrit = int(m.group(1)) if m else None
    taux_auto = bool(re.search(r"taux\s*auto", t))
    reste = re.sub(r"(cap|taux)\s*(\d+|auto)", " ", t)
    code = None
    for mot in reste.split()[1:]:
        if (re.fullmatch(r"[a-z0-9-]{3,7}", mot) and re.search(r"[a-z]", mot)
                and not re.fullmatch(r"\d+(ft|kt|m|nm)", mot)
                and mot not in ("pieds", "vers", "pour", "cap", "taux", "auto")):
            code = mot.upper()
            reste = reste.replace(mot, " ", 1)
            break
    n = [int(x) for x in re.findall(r"\d+", reste)]
    if not n:
        n = [None, None]                # aucune altitude : croisiere du controle ou du dernier vol
    elif code:
        if not 1000 <= n[0] <= 45000:
            return None
        n.insert(1, None)
    elif n and 1000 <= n[0] <= 45000 and (len(n) == 1 or n[1] <= 360):
        n.insert(1, None)               # une seule altitude : l'arrivee sera lue dans le plan de vol
    elif len(n) < 2 or not (500 <= n[1] < n[0] <= 45000):
        return None
    taux = taux_ecrit if taux_ecrit and 360 < taux_ecrit <= 6000 else None
    for x in n[2:]:
        if x <= 360 and cap is None:
            cap = x
        elif 360 < x <= 6000:
            taux = x
    if cap is not None and not 0 <= cap <= 360:
        return None
    if taux_auto:
        taux = "auto"
    return {"croisiere": n[0], "approche": n[1], "cap": None if cap is None else cap % 360, "taux": taux,
            "arrivee": code, "dest": None, "terrain": None, "code": None, "alt_terrain": None}


def completer_vol(demande):
    """Pour un vol donne avec un code d'aeroport : trouve le terrain et calcule l'altitude
    de remise. Renvoie None si tout va bien, sinon le message a envoyer."""
    code = demande["arrivee"]
    try:
        ap = chercher_aeroport(code)
    except Exception:
        return ("Je n'ai pas pu telecharger la liste des aeroports (connexion internet ?). "
                "Donne l'altitude de remise toi-meme : vol 9000 3300")
    if ap is None:
        return (f"Je ne connais pas l'aeroport {code}. Verifie le code, ou donne l'altitude "
                "de remise toi-meme : vol 9000 3300")
    if ap["alt_ft"] is None:
        return (f"Je connais {code} ({ap['nom']}) mais pas son altitude. Donne l'altitude "
                "de remise toi-meme : vol 9000 3300")
    approche = int(round((ap["alt_ft"] + HAUTEUR_REMISE_M / 0.3048) / 100) * 100)
    if approche >= demande["croisiere"]:
        return (f"{code} ({ap['nom']}) est a {ap['alt_ft']:.0f} ft : je te rendrais l'avion a {approche} ft, "
                f"au-dessus de ta croisiere de {demande['croisiere']} ft. Choisis une croisiere plus haute.")
    demande["approche"], demande["dest"] = approche, (ap["lat"], ap["lon"])
    demande["code"], demande["alt_terrain"] = ap["code"], ap["alt_ft"]
    demande["terrain"] = f"{ap['code']} ({ap['nom']}, terrain a {ap['alt_ft']:.0f} ft)"
    return None


def texte_sim(v):
    """Texte lu dans le simulateur (la bibliotheque le rend en octets)."""
    if isinstance(v, bytes):
        v = v.decode("utf-8", "ignore")
    return v.strip().strip("\x00").upper() if isinstance(v, str) else ""


def arrivee_du_plan(sim):
    """Aeroport d'arrivee lu dans le plan de vol du GPS, ou None s'il n'est pas (encore)
    lisible. Le simulateur ne donne que le prochain point du plan : on ne sait donc que
    c'est l'arrivee qu'une fois sur la derniere branche (ou si une approche est chargee)."""
    code = texte_sim(sim.lire("GPS_APPROACH_AIRPORT_ID"))
    if code:
        ap = chercher_aeroport(code)
        if ap:
            return ap
    if not sim.lire("GPS_IS_ACTIVE_FLIGHT_PLAN"):
        return None
    nb, i = sim.lire("GPS_FLIGHT_PLAN_WP_COUNT"), sim.lire("GPS_FLIGHT_PLAN_WP_INDEX")
    if nb is None or i is None or num(nb) < 2 or num(i) < num(nb) - 1:
        return None
    lat, lon = sim.lire("GPS_WP_NEXT_LAT"), sim.lire("GPS_WP_NEXT_LON")
    point = (num(lat), num(lon)) if lat is not None and lon is not None and (num(lat) or num(lon)) else None
    code = texte_sim(sim.lire("GPS_WP_NEXT_ID"))
    ap = chercher_aeroport(code) if code else None
    # Un point de route peut porter le meme nom qu'un aeroport lointain : on verifie la position.
    if ap and point and distance_nm(point[0], point[1], ap["lat"], ap["lon"]) > 5:
        ap = None
    if ap is None and point:
        ap = chercher_aeroport(pres_de=point)
    return ap


def fixer_arrivee(a, ap, origine):
    """Regle l'altitude de remise d'apres l'aeroport trouve. Renvoie le message a envoyer."""
    if ap["alt_ft"] is None:
        return (f"Arrivee {ap['code']} ({ap['nom']}) {origine}, mais je ne connais pas son altitude. "
                "Envoie 'stop' puis donne l'altitude toi-meme : vol 9000 3300")
    approche = int(round((ap["alt_ft"] + HAUTEUR_REMISE_M / 0.3048) / 100) * 100)
    if approche >= a.croisiere:
        return (f"Arrivee {ap['code']} ({ap['nom']}) {origine} : le terrain est a {ap['alt_ft']:.0f} ft, je te "
                f"rendrais l'avion a {approche} ft, au-dessus de ta croisiere. Envoie 'altitude ...' pour monter.")
    a.approche, a.dest = approche, (ap["lat"], ap["lon"])
    a.code_arrivee, a.alt_terrain = ap["code"], ap["alt_ft"]
    a.terrain =f"{ap['code']} ({ap['nom']}, terrain a {ap['alt_ft']:.0f} ft)"
    return (f"Arrivee {a.terrain} {origine}. Je te rends l'avion {HAUTEUR_REMISE_M} m au-dessus, "
            f"a {approche} ft.")


def traiter_commandes(sim, texte_etat, ordres):
    """Repond aux messages Telegram recus. Les ordres qui changent le vol sont
    notes dans 'ordres', et c'est la boucle de vol qui les execute."""
    en_vol = ordres.get("en_vol")
    for brut in TG.commandes():
        mot, n, texte = analyser(brut)
        if mot in ("etat", "status", "statut", "position", "ou", "?", "temps"):
            TG.envoyer(texte_etat())
            pos = position(sim) if sim else None
            if pos:
                TG.position(*pos)
        elif mot == "vol":
            if en_vol:
                TG.envoyer("Un vol est deja en cours. Envoie 'stop' d'abord pour en preparer un autre.")
            elif lire_vol(texte) is None:
                TG.envoyer("Ecris par exemple : vol 9000 LFMD (croisiere, code de l'aeroport d'arrivee) "
                           "ou vol 9000 3300 (croisiere, altitude de remise plus basse). "
                           "Ajoute le cap a la fin si tu en veux un.")
            else:
                demande = lire_vol(texte)
                garde = lire_reglages()
                # Croisiere non ecrite : on suivra le controle, avec celle du dernier vol en secours.
                demande["controle"] = demande["croisiere"] is None
                demande["croisiere"] = demande["croisiere"] or garde.get("croisiere")
                demande["taux"] = demande["taux"] or garde.get("taux") or "auto"
                if demande["croisiere"] is None:
                    TG.envoyer("Pour ce premier vol, donne-moi la croisiere : vol 30000 taux 1800. "
                               "Je la retiendrai, et 'vol' tout court suffira ensuite.")
                    continue
                refus = completer_vol(demande) if demande["arrivee"] else None
                if refus:
                    TG.envoyer(refus)
                else:
                    garder_reglages(croisiere=None if demande["controle"] else demande["croisiere"],
                                    taux=demande["taux"])
                    ordres["vol"] = demande
        elif mot in ("pause", "reprendre", "reprise", "continuer"):
            if not sim:
                TG.envoyer("Je ne suis pas encore connecte au simulateur.")
            elif mot == "pause":
                ok = sim.envoyer("PAUSE_ON") or sim.envoyer("PAUSE_SET", 1)
                TG.envoyer("Simulateur en pause." if ok else "Je n'ai pas reussi a mettre en pause.")
            else:
                ok = sim.envoyer("PAUSE_OFF") or sim.envoyer("PAUSE_SET", 0)
                TG.envoyer("Pause enlevee, le vol reprend." if ok else "Je n'ai pas reussi a enlever la pause.")
        elif mot in ("arrivee", "destination", "aeroport"):
            code = texte.split()[1].upper() if len(texte.split()) > 1 else ""
            if not en_vol:
                TG.envoyer("Pas de vol en cours. Envoie d'abord : vol 9000")
            elif not re.fullmatch(r"[A-Z0-9-]{3,7}", code):
                TG.envoyer("Ecris par exemple : arrivee LFMD")
            else:
                ordres["arrivee"] = code
        elif mot == "suivi":
            minutes = n[0] if n else 0
            garder_reglages(suivi=minutes or "non")
            ordres["suivi"] = minutes
            TG.envoyer(f"Je t'enverrai l'etat du vol toutes les {minutes} minutes." if minutes
                       else "Suivi automatique arrete. Envoie 'etat' quand tu veux.")
        elif mot in ("atterrissage", "atterrir", "atterris", "atterrisage"):
            mots = texte.split()[1:]
            if any(m in mots for m in ("oui", "on", "auto")):
                garder_reglages(atterrissage="oui")
                ordres["atterrissage"] = True
                TG.envoyer("Atterrissage automatique active : je fais l'approche, je me pose et je m'arrete sur "
                           "la piste. Il me faut le code de l'aeroport d'arrivee et, la premiere fois avec un "
                           "avion, qu'il soit a l'arret au sol quand tu envoies 'vol' (je mesure ses gouvernes "
                           "et je les retiens). Envoie 'repetition' en croisiere pour essayer en plein ciel.")
            elif any(m in mots for m in ("non", "off")):
                garder_reglages(atterrissage="non")
                ordres["atterrissage"] = False
                TG.envoyer("Atterrissage automatique coupe : je te rendrai l'avion a 10 NM, comme avant.")
            else:
                actif = lire_reglages().get("atterrissage") == "oui"
                TG.envoyer(f"Atterrissage automatique {'active' if actif else 'coupe'}. "
                           "Envoie 'atterrissage oui' ou 'atterrissage non'.")
        elif mot in ("repetition", "repeter", "repete", "entrainement"):
            if not en_vol:
                TG.envoyer("Pas de vol en cours : la repetition se fait en vol, en montee ou en croisiere.")
            else:
                ordres["repetition"] = True
        elif mot == "piste":
            mots = texte.split()
            if len(mots) > 1 and re.fullmatch(r"\d{1,2}[lcr]?", mots[1]):
                ordres["piste"] = mots[1].upper()
                TG.envoyer(f"Piste {mots[1].upper()} notee pour l'arrivee.")
            else:
                TG.envoyer("Ecris par exemple : piste 22 ou piste 04L (sinon je prends la plus face au vent)")
        elif mot in ("prepare", "preparer", "preparation", "prepa"):
            ordres["prepare"] = True
        elif mot in ("moteur", "moteurs", "demarre", "demarrer", "demarrage"):
            ordres["moteur"] = True
        elif mot in ("altitude", "descente", "stop", "cap", "route", "decollage"):
            if not en_vol:
                TG.envoyer("Pas de vol en cours. Envoie d'abord : vol 9000 3300")
            elif mot == "decollage":
                ordres["decollage"] = n[0] if n and 40 <= n[0] <= 200 else True
            elif mot == "altitude":
                if n and 500 <= n[0] <= 45000:
                    ordres["altitude"] = n[0]
                else:
                    TG.envoyer("Ecris par exemple : altitude 11000")
            elif mot == "cap":
                if n and 0 <= n[0] <= 360:
                    ordres["cap"] = n[0] % 360
                else:
                    TG.envoyer("Ecris par exemple : cap 270 (entre 0 et 360)")
            else:
                ordres[mot] = True
        else:
            TG.envoyer(AIDE)


# --------------------------------------------------------------------------
# Outils
# --------------------------------------------------------------------------
def num(v, defaut=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return defaut


def booleen(v):
    """True, False, ou None quand le simulateur n'a pas repondu (a ne pas confondre avec False)."""
    if v is None:
        return None
    try:
        return float(v) >= 0.5
    except (TypeError, ValueError):
        return None


def dire(msg, alerte=False, tel=None):
    """Affiche dans le terminal. Les alertes bipent et partent aussi sur Telegram
    (tel=True force l'envoi, tel=False l'empeche). Une meme alerte ne revient
    pas plus d'une fois par minute."""
    if alerte:
        cle = msg[:30]
        if time.time() - _derniers.get(cle, 0) < 60:
            return
        _derniers[cle] = time.time()
    ligne = time.strftime("%H:%M:%S ") + ("!! " if alerte else "   ") + msg
    print(ligne, flush=True)
    try:
        if os.path.getsize(JOURNAL) > 2_000_000:          # au-dela de 2 Mo, on garde une seule archive
            os.replace(JOURNAL, JOURNAL[:-4] + "_ancien.txt")
    except OSError:
        pass
    try:
        with open(JOURNAL, "a", encoding="utf-8") as f:
            f.write(time.strftime("%Y-%m-%d ") + ligne + "\n")
    except Exception:
        pass
    if alerte if tel is None else tel:
        TG.envoyer(msg)
    if alerte:
        try:
            import winsound
            winsound.Beep(1200, 400)
        except Exception:
            print("\a", end="")


def altitude(sim):
    """Altitude lue a l'altimetre, celle que suit le pilote automatique (l'altitude
    vraie peut en differer de plusieurs centaines de pieds selon la meteo)."""
    v = sim.lire("INDICATED_ALTITUDE")
    return num(v if v is not None else sim.lire("PLANE_ALTITUDE"))


def position(sim, vite=False):
    """Latitude et longitude en degres, ou None."""
    lire = sim.lire_vite if vite else sim.lire
    lat, lon = num(lire("PLANE_LATITUDE"), None), num(lire("PLANE_LONGITUDE"), None)
    if lat is None or lon is None:
        return None
    if abs(lat) <= 1.6 and abs(lon) <= 3.2:      # la bibliotheque renvoie des radians
        lat, lon = math.degrees(lat), math.degrees(lon)
    return lat, lon


def distance_restante(sim, dest):
    """Distance a la destination en NM, ou None si inconnue."""
    if dest:
        lat, lon = num(sim.lire("PLANE_LATITUDE"), None), num(sim.lire("PLANE_LONGITUDE"), None)
        if lat is None or lon is None:
            return None
        # La bibliotheque renvoie des radians ; on convertit si ce sont des degres.
        if abs(lat) > 1.6 or abs(lon) > 3.2:
            lat, lon = math.radians(lat), math.radians(lon)
        la2, lo2 = math.radians(dest[0]), math.radians(dest[1])
        a = math.sin((la2 - lat) / 2) ** 2 + math.cos(lat) * math.cos(la2) * math.sin((lo2 - lon) / 2) ** 2
        return 2 * 6371000 * math.asin(math.sqrt(a)) / NM
    if not sim.lire("GPS_IS_ACTIVE_FLIGHT_PLAN"):
        return None
    ete, gs = num(sim.lire("GPS_ETE"), None), num(sim.lire("GPS_GROUND_SPEED"), None)
    if ete is None or gs is None or ete <= 0:
        return None
    return ete * gs / NM


def estimation(sim, dist, remise):
    """Texte du temps restant, d'apres la distance et la vitesse sol actuelle."""
    gs_kt = num(sim.lire("GPS_GROUND_SPEED")) * 1.944
    if dist is None or gs_kt < 40:
        return "temps restant inconnu"
    minutes = dist / gs_kt * 60
    heure = time.strftime("%H:%M", time.localtime(time.time() + minutes * 60))
    texte = f"arrivee dans {int(minutes // 60)} h {int(minutes % 60):02d} (vers {heure})"
    avant_remise = (dist - remise) / gs_kt * 60
    if avant_remise > 1:
        texte += f", a toi dans {avant_remise:.0f} min"
    return texte


def regler_vs(sim, vs):
    """Regle le taux vertical du pilote automatique et verifie ce que l'avion affiche.
    Un taux negatif envoye directement est mal compris par certains avions (il devient un
    grand taux positif) : on part donc de zero et on descend cran par cran, en relisant
    l'affichage. Renvoie le taux affiche a la fin, ou None s'il est illisible."""
    vs = int(round(vs / 100) * 100)
    sim.envoyer("AP_VS_VAR_SET_ENGLISH", max(vs, 0))
    affiche = None
    for _ in range(5):
        sim.attendre(0.4)
        lu = sim.lire_vite("AUTOPILOT_VERTICAL_HOLD_VAR")
        if lu is None:
            break
        affiche = num(lu)
        ecart = vs - affiche
        if abs(ecart) < 60:
            return affiche
        for _ in range(min(int(round(abs(ecart) / 100)), 60)):
            sim.envoyer("AP_VS_VAR_INC" if ecart > 0 else "AP_VS_VAR_DEC")
    if affiche is None and vs < 0:          # affichage illisible : on descend a l'aveugle depuis zero
        for _ in range(min(-vs // 100, 60)):
            sim.envoyer("AP_VS_VAR_DEC")
    return affiche


def engager_vertical(sim, altitude, vs):
    """Affiche l'altitude visee, engage le mode VS et regle le taux. Note dans le journal
    ce que le pilote automatique affiche reellement, pour pouvoir verifier apres coup."""
    sim.envoyer("AP_ALT_VAR_SET_ENGLISH", altitude)
    # Tous les avions n'obeissent pas a la meme commande : on en essaie plusieurs, en relisant
    # le mode apres chacune. AP_VS_HOLD est une bascule : seulement si le mode est lu coupe.
    for i, (ev, val) in enumerate((("AP_PANEL_VS_ON", None), ("AP_VS_ON", None), ("AP_PANEL_VS_SET", 1),
                                   ("AP_VS_SET", 1), ("AP_VS_HOLD", None))):
        mode = booleen(frais(sim, "AUTOPILOT_VERTICAL_HOLD"))
        if mode is True or (mode is None and i > 0):
            break
        if sim.envoyer(ev, val):
            # Certains avions recalent le taux sur la vitesse verticale du moment quand le
            # mode VS s'engage : on attend qu'il soit engage avant de donner le taux voulu.
            sim.attendre(1)
    affiche = regler_vs(sim, vs)
    alt_affichee = sim.lire_vite("AUTOPILOT_ALTITUDE_LOCK_VAR")
    mode = booleen(sim.lire_vite("AUTOPILOT_VERTICAL_HOLD"))
    dire(f"Pilote automatique : demande {altitude} ft a {vs:+.0f} ft/min ; il affiche "
         f"{'?' if alt_affichee is None else f'{num(alt_affichee):.0f}'} ft a "
         f"{'?' if affiche is None else f'{affiche:+.0f}'} ft/min, mode VS "
         f"{'engage' if mode else 'non engage' if mode is False else 'illisible'}.", tel=False)
    if mode is False:
        dire("Cet avion refuse toutes mes commandes de mode VS : engage le mode VS toi-meme "
             "(je regle deja l'altitude et le taux).", alerte=True)


def cap_route(sim):
    """Cap magnetique vers le prochain point du plan de vol, en degres, ou None."""
    if not sim.lire("GPS_IS_ACTIVE_FLIGHT_PLAN"):
        return None
    b = num(sim.lire("GPS_WP_BEARING"), None)
    if b is None or b == 0:           # zero pile : le GPS n'a plus de point a viser (plan termine ou absent)
        return None
    if abs(b) <= 6.4:                 # la bibliotheque renvoie des radians
        b = math.degrees(b)
    return round(b) % 360


def mode_cap(sim, cap):
    """Regle le cap et engage le maintien de cap, en essayant plusieurs commandes."""
    sim.envoyer("HEADING_BUG_SET", cap)
    for ev in ("AP_HDG_HOLD_ON", "AP_PANEL_HEADING_ON", "AP_HDG_HOLD"):
        if booleen(sim.lire("AUTOPILOT_HEADING_LOCK")) is not False:
            return                    # mode cap engage, ou lecture manquee : on ne touche a rien
        sim.envoyer(ev)
        sim.attendre(1)


def engager_lateral(sim, a):
    """Renvoie 'CAP' (le script vise chaque point du plan de vol), 'NAV' ou 'FIXE'."""
    cap = cap_route(sim)
    if cap is not None and not a.nav:
        mode_cap(sim, cap)
        dire(f"Lateral : je vise les points du plan de vol au cap (cap {cap:03d}).")
        return "CAP"
    if sim.lire("GPS_IS_ACTIVE_FLIGHT_PLAN"):
        if not sim.lire("GPS_DRIVES_NAV1"):
            sim.envoyer("TOGGLE_GPS_DRIVES_NAV1")
        sim.envoyer("AP_NAV1_HOLD_ON")
        dire("Lateral : suivi du plan de vol par le mode NAV de l'avion.")
        return "NAV"
    cap = round(math.degrees(num(sim.lire("PLANE_HEADING_DEGREES_MAGNETIC")))) % 360
    mode_cap(sim, cap)
    dire(f"Lateral : pas de plan de vol actif, maintien du cap {cap:03d}.", alerte=True)
    return "FIXE"


def tenir_vitesse(sim, etat, ias):
    """Correcteur simple : bouge la manette selon l'ecart et la tendance de vitesse."""
    etat["gaz"] += 0.004 * (etat["cible"] - ias) - 0.02 * (ias - etat["prec"])
    etat["gaz"] = min(max(etat["gaz"], 0.0), 1.0)
    etat["prec"] = ias
    sim.envoyer("THROTTLE_SET", etat["gaz"] * 16383)


# --------------------------------------------------------------------------
# Preparation de l'avion et decollage automatique
# --------------------------------------------------------------------------
def frais(sim, nom):
    """Lecture rapide d'une valeur a jour (la bibliotheque rend d'abord l'ancienne)."""
    sim.lire_vite(nom)
    sim.attendre(0.15)
    return sim.lire_vite(nom)


def desserrer_frein(sim):
    """Desserre le frein de parking. Renvoie True s'il est desserre, False s'il resiste, None si illisible."""
    for _ in range(2):
        etat = booleen(frais(sim, "BRAKE_PARKING_POSITION"))
        if etat is not True:
            return None if etat is None else True
        sim.envoyer("PARKING_BRAKES")
        sim.attendre(0.5)
    return booleen(frais(sim, "BRAKE_PARKING_POSITION")) is False


def enlever_protections(sim):
    """Cales, caches et flammes de MSFS 2024. Renvoie (enlevees, restantes)."""
    enlevees = restantes = 0
    for i in range(8):
        if num(sim.variable(f"COVER ON:{i}")) < 0.5:
            continue
        if num(sim.variable(f"COVER ON:{i}", valeur=0), 1) < 0.5:
            enlevees += 1
        else:
            restantes += 1
    return enlevees, restantes


def preparer(sim):
    """Prepare l'avion pour le roulage et dit sur Telegram ce qui a ete fait."""
    lignes = []
    enlevees, restantes = enlever_protections(sim)
    if restantes:
        lignes.append(f"Protections : {restantes} refusent de partir, enleve-les a la main "
                      "(tour de l'avion)." + (f" {enlevees} enlevee(s)." if enlevees else ""))
    elif enlevees:
        lignes.append(f"Protections : {enlevees} enlevee(s) (cales, caches).")
    else:
        lignes.append("Protections : je n'en vois aucune en place. S'il en reste, enleve-les a la main.")

    # Moteur : je ne le demarre que si je suis sur qu'il ne reste aucune protection,
    # donc seulement quand j'ai pu en voir et en enlever moi-meme.
    moteur = moteurs_en_marche(sim)
    if moteur is False:
        if restantes:
            lignes.append("Moteur : arrete. Je ne le demarre pas tant qu'il reste des protections.")
        elif not enlevees:
            lignes.append("Moteur : arrete. Comme je ne vois pas les protections de cet avion, verifie "
                          "qu'il n'en reste aucune puis envoie 'moteur'.")
        else:
            TG.envoyer("Protections enlevees, je demarre le moteur (jusqu'a deux minutes).")
            moteur = demarrer_moteur(sim, seul=False)
            lignes.append("Moteur : demarre." if moteur else
                          "Moteur : il n'a pas demarre, fais-le a la main.")

    if num(frais(sim, "FLAPS_HANDLE_INDEX")) < 0.5:
        sim.envoyer("FLAPS_1")
        sim.attendre(1)
    cran = frais(sim, "FLAPS_HANDLE_INDEX")
    lignes.append("Volets : premier cran (decollage)." if cran is None or num(cran) >= 0.5 else
                  "Volets : cet avion n'obeit pas a ma commande, sors-les a la main si tu en veux.")

    for bascule, etat in (("TOGGLE_BEACON_LIGHTS", "LIGHT_BEACON"), ("TOGGLE_NAV_LIGHTS", "LIGHT_NAV"),
                          ("TOGGLE_TAXI_LIGHTS", "LIGHT_TAXI")):
        if booleen(frais(sim, etat)) is False:      # jamais de bascule a l'aveugle
            sim.envoyer(bascule)
    sim.envoyer("STROBES_ON")
    sim.envoyer("LANDING_LIGHTS_ON")
    sim.envoyer("PITOT_HEAT_ON")
    lignes.append("Feux et rechauffage pitot : allumes.")

    if not moteur:
        lignes.append("Frein de parking : laisse serre tant que le moteur est arrete. "
                      "Renvoie 'prepare' une fois le moteur en marche.")
    else:
        frein = desserrer_frein(sim)
        lignes.append("Frein de parking : desserre." if frein else
                      "Frein de parking : je n'arrive pas a le desserrer, fais-le a la main." if frein is False
                      else "Frein de parking : etat illisible, verifie-le.")
    pret = moteur and not restantes
    dire(("Avion pret a rouler. Aligne-toi sur la piste puis envoie 'decollage' "
          "(ou decolle toi-meme).\n" if pret else "Preparation incomplete.\n") + "\n".join(lignes), tel=True)


def demarrer_moteur(sim, seul=True):
    """Demarrage automatique du simulateur, verifie en relisant l'etat du moteur.
    Renvoie True si le moteur tourne. 'seul' : demande directe, on repond sur Telegram."""
    def reponse(msg, **k):
        if seul:
            dire(msg, **k)

    if moteurs_en_marche(sim) is True:
        reponse("Le moteur tourne deja.", tel=True)
        return True
    if any(num(sim.variable(f"COVER ON:{i}")) >= 0.5 for i in range(8)):
        reponse("Je ne demarre pas : il reste des protections sur l'avion. Enleve-les d'abord "
                "(envoie 'prepare' ou fais le tour de l'avion).", tel=True)
        return False
    if not sim.envoyer("ENGINE_AUTO_START"):
        reponse("Cet avion ne connait pas le demarrage automatique : demarre le moteur a la main.", tel=True)
        return False
    reponse("Demarrage du moteur en cours, ca peut prendre deux minutes.", tel=True)
    for _ in range(60):
        sim.attendre(2)
        if moteurs_en_marche(sim) is True:
            reponse("Moteur demarre. Le frein de parking est toujours serre : envoie 'prepare' pour le "
                    "desserrer quand tu es pret a rouler.", tel=True)
            return True
    reponse("Le moteur n'a pas demarre en deux minutes : demarre-le a la main.", alerte=True)
    return False


class Rentree:
    """Rentre le train ou les volets. Tous les avions n'obeissent pas a la meme commande :
    on en essaie plusieurs, l'une apres l'autre, en verifiant a chaque fois sur la manette
    que c'est bien rentre. A appeler environ une fois par seconde, avion en l'air."""

    def __init__(self, nom, rentre, est_sorti, commandes, fixe=None):
        self.nom, self.mot_rentre = nom, rentre
        self.est_sorti, self.commandes, self.fixe = est_sorti, commandes, fixe
        self.fini, self.rentre = False, None       # rentre : True confirme, False echec, None inconnu
        self.essai, self.attente = 0, 0

    def pas(self, sim):
        if self.fini:
            return
        if self.essai == 0 and self.fixe and self.fixe(sim):
            self.fini = True                       # train fixe : rien a faire
            return
        sorti = self.est_sorti(sim)
        if self.attente > 0:                       # on laisse a la commande le temps d'agir
            self.attente -= 1
            return
        if sorti is False:
            self.fini, self.rentre = True, True
            if self.essai:                         # deja rentre avant que j'y touche : je ne dis rien
                dire(f"{self.nom} {self.mot_rentre}"
                     + (f" (a la {self.essai}e commande essayee)." if self.essai > 1 else "."), tel=True)
            return
        if sorti is None and self.essai >= 2:      # manette illisible : on ne peut pas verifier
            self.fini = True
            dire(f"{self.nom} : commandes envoyees, mais je ne peux pas verifier le resultat. "
                 "Jette un oeil.", tel=True)
            return
        if self.essai >= len(self.commandes):
            self.fini, self.rentre = True, False
            dire(f"{self.nom} : cet avion n'obeit a aucune de mes commandes, a toi de le faire a la main.",
                 alerte=True)
            return
        self.commandes[self.essai](sim)
        self.essai += 1
        self.attente = 2


def Train():
    return Rentree(
        "Train", "rentre",
        lambda sim: booleen(sim.lire_vite("GEAR_HANDLE_POSITION")),
        [lambda sim: sim.envoyer("GEAR_UP"),
         lambda sim: sim.envoyer("GEAR_SET", 0),
         lambda sim: sim.variable("GEAR HANDLE POSITION", "Bool", 0),
         lambda sim: sim.envoyer("GEAR_TOGGLE")],          # la manette est lue sortie : la bascule la remonte
        fixe=lambda sim: booleen(sim.lire("IS_GEAR_RETRACTABLE")) is False)


def Volets():
    def sortis(sim):
        cran = sim.lire_vite("FLAPS_HANDLE_INDEX")
        return None if cran is None else num(cran) >= 0.5

    def cran_par_cran(sim):
        for _ in range(6):
            sim.envoyer("FLAPS_DECR")

    return Rentree(
        "Volets", "rentres", sortis,
        [lambda sim: sim.envoyer("FLAPS_UP"),
         lambda sim: sim.envoyer("FLAPS_SET", 0),
         lambda sim: sim.variable("FLAPS HANDLE INDEX", "Number", 0),
         cran_par_cran])


def sens_commande(sim, reglage, bouton, variable):
    """Le simulateur ne dit pas dans quel sens agit une commande de gouverne. On le
    mesure a l'arret : +1 si une valeur positive de 'reglage' agit comme 'bouton'
    (palonnier a droite, profondeur a cabrer), -1 si c'est l'inverse, None si illisible."""
    sim.envoyer(reglage, 0)
    sim.attendre(0.4)
    p0 = frais(sim, variable)
    sim.envoyer(reglage, 5000)
    sim.attendre(0.4)
    p1 = frais(sim, variable)
    sim.envoyer(reglage, 0)
    sim.attendre(0.4)
    p2 = frais(sim, variable)
    for _ in range(4):
        sim.envoyer(bouton)
    sim.attendre(0.4)
    p3 = frais(sim, variable)
    sim.envoyer(reglage, 0)
    if None in (p0, p1, p2, p3):
        return None
    par_reglage, par_bouton = num(p1) - num(p0), num(p3) - num(p2)
    if abs(par_reglage) < 0.02 or abs(par_bouton) < 0.005:
        return None
    return 1 if par_reglage * par_bouton > 0 else -1


def loi_palonnier(ecart, taux, ias, douceur=1.0):
    """Palonnier au sol (decollage et freinage apres l'atterrissage) : revient sur l'axe ('ecart'
    en degres, positif pour tourner a droite) en freinant la rotation du nez ('taux' en deg/s).
    Le gain baisse avec la vitesse, car la gouverne devient plus efficace."""
    gain = 900 * douceur * min(1.0, 50 / max(ias, 50))
    return max(-9000, min(9000, gain * ecart - 900 * douceur * taux))


def decoller(sim, a, ordres, vr, train):
    """Decollage automatique, avion aligne sur la piste. Renvoie True quand l'avion
    est en l'air avec le pilote automatique engage, False si le decollage n'a pas eu lieu."""
    def refus(raison):
        dire(f"Decollage automatique impossible : {raison} A toi le decollage.", alerte=True)
        return False

    if booleen(frais(sim, "SIM_ON_GROUND")) is not True:
        return refus("l'avion n'est pas au sol.")
    if moteurs_en_marche(sim) is False:
        return refus("un moteur est arrete.")
    if num(frais(sim, "GROUND_VELOCITY")) > 10:
        return refus("l'avion roule deja, arrete-toi aligne sur la piste.")
    sens = getattr(a, "sens", {})                   # deja mesure a l'arret si l'atterrissage est demande
    s_pal = sens.get("pal") or sens_commande(sim, "RUDDER_SET", "RUDDER_RIGHT", "RUDDER_POSITION")
    s_prof = sens.get("prof") or sens_commande(sim, "ELEVATOR_SET", "ELEV_UP", "ELEVATOR_POSITION")
    if s_pal is None or s_prof is None:
        return refus("cet avion ne me laisse pas lire ses gouvernes.")
    axe = frais(sim, "PLANE_HEADING_DEGREES_TRUE")
    cap_piste = frais(sim, "PLANE_HEADING_DEGREES_MAGNETIC")
    if axe is None or cap_piste is None:
        return refus("je ne lis pas le cap de l'avion.")
    axe, cap_piste = math.degrees(num(axe)), round(math.degrees(num(cap_piste))) % 360

    if num(frais(sim, "FLAPS_HANDLE_INDEX")) < 0.5:
        sim.envoyer("FLAPS_1")
    if desserrer_frein(sim) is False:
        return refus("le frein de parking ne se desserre pas.")
    dire(f"Decollage : axe de piste {cap_piste:03d}, plein gaz, je cabre a {vr} kt. "
         "Envoie 'stop' pour interrompre avant l'envol.", tel=True)

    mesures = []
    try:
        return _roulage_et_envol(sim, a, ordres, vr, axe, cap_piste, s_pal, s_prof, mesures, train)
    finally:
        enregistrer_mesures(mesures, "decollage", "t;vitesse_kt;ecart_axe_deg;rotation_nez_deg_s;palonnier;"
                                                  "douceur;profondeur;vs_ft_min;hauteur_ft;au_sol")


def enregistrer_mesures(mesures, quoi, entete):
    """Garde les mesures d'un decollage ou d'un atterrissage dans un fichier, pour pouvoir regler le bot ensuite."""
    if not mesures:
        return
    try:
        nom = os.path.join(os.path.dirname(JOURNAL), time.strftime(f"{quoi}_%Y%m%d_%H%M%S.csv"))
        with open(nom, "w", encoding="utf-8") as f:
            f.write(entete + "\n")
            for m in mesures:
                f.write(";".join(str(x) for x in m) + "\n")
    except Exception:
        pass


def _roulage_et_envol(sim, a, ordres, vr, axe, cap_piste, s_pal, s_prof, mesures, train):
    # Le temps est mesure a l'horloge : chaque lecture du simulateur prend quelques
    # centiemes de seconde, la boucle ne tourne donc pas a cadence fixe.
    t0 = t_prec = sim.horloge()
    t_msg = t_ap = 0.0
    t_train = None
    t_rotation = t_envol = None
    cap, cap_prec, taux = axe, axe, 0.0
    ias, vs, agl, au_sol = 0.0, 0.0, 0.0, True
    palonnier = profondeur = 0.0
    autres = ["AIRSPEED_INDICATED", "SIM_ON_GROUND", "VERTICAL_SPEED", "PLANE_ALT_ABOVE_GROUND"]
    n = 0
    # Si l'avion zigzague autour de l'axe, on adoucit le palonnier : a chaque passage
    # franc d'un cote a l'autre de l'axe, le gain baisse (jamais sous 30 %).
    douceur, cote = 1.0, 0
    while True:
        sim.attendre(0.02)
        maintenant = sim.horloge()
        dt, t_prec, t = max(maintenant - t_prec, 0.005), maintenant, maintenant - t0
        t_msg += dt
        # Le cap est lu a chaque tour, les autres mesures a tour de role (pour aller vite).
        # Une lecture manquee garde la valeur d'avant : on ne pilote jamais sur un zero invente.
        v = sim.lire_vite("PLANE_HEADING_DEGREES_TRUE")
        if v is not None:
            cap = math.degrees(num(v))
        nom = autres[n % 4]
        n += 1
        v = sim.lire_vite(nom)
        if v is not None:
            if nom == "AIRSPEED_INDICATED":
                ias = num(v)
            elif nom == "VERTICAL_SPEED":
                vs = num(v)
            elif nom == "PLANE_ALT_ABOVE_GROUND":
                agl = num(v)
            elif booleen(v) is not None:
                au_sol = booleen(v)
        if not au_sol and t_envol is None:
            t_envol = t

        # Palonnier, au sol seulement : on revient sur l'axe de piste, en freinant la rotation
        # du nez. Une fois en l'air il est ramene au neutre (y tenir le cap fait osciller l'avion).
        ecart = (axe - cap + 180) % 360 - 180            # positif : il faut tourner a droite
        taux += (((cap - cap_prec + 180) % 360 - 180) / dt - taux) * min(dt / 0.5, 1.0)
        cap_prec = cap
        if au_sol:
            if abs(ecart) > 1.5:
                nouveau = 1 if ecart > 0 else -1
                if cote and nouveau != cote:
                    douceur = max(douceur * 0.7, 0.3)
                cote = nouveau
            palonnier = loi_palonnier(ecart, taux, ias, douceur)
        else:
            palonnier *= max(1 - dt / 0.5, 0.0)
        sim.envoyer("RUDDER_SET", s_pal * palonnier)
        mesures.append((round(t, 2), round(ias), round(ecart, 2), round(taux, 2), round(palonnier),
                        round(douceur, 2), round(profondeur, 2), round(vs), round(agl), int(au_sol)))

        if t <= 3.5:                                      # mise des gaz progressive
            sim.envoyer("THROTTLE_SET", 16383 * min(t / 3, 1))

        # Arret du decollage, seulement tant qu'on est loin de la vitesse d'envol.
        if t_msg >= 1:
            t_msg = 0
            traiter_commandes(sim, lambda: f"Decollage en cours : {ias:.0f} kt.", ordres)
        raison = None
        if au_sol and t_rotation is None and ias < vr - 10:
            if ordres.pop("stop", None):
                raison = "tu as envoye stop"
            elif abs(ecart) > 20:
                raison = "l'avion sort de l'axe de piste"
            elif t > 25 and ias < 40:
                raison = "l'avion n'accelere pas"
        if raison:
            sim.envoyer("THROTTLE_SET", 0)
            fin_freinage = sim.horloge() + 20
            while sim.horloge() < fin_freinage:
                sim.envoyer("BRAKES")
                sim.attendre(0.1)
                if num(sim.lire_vite("GROUND_VELOCITY"), 99) < 3:
                    break
            sim.envoyer("RUDDER_SET", 0)
            dire(f"Decollage interrompu ({raison}) : gaz reduits, avion freine. A toi.", alerte=True)
            return False

        # Rotation, puis tenue d'une montee de 1000 a 2000 ft/min a la profondeur.
        if t_rotation is None and ias >= vr:
            t_rotation = t
        if t_rotation is not None:
            if au_sol:
                profondeur = min(profondeur + 0.23 * dt, 0.35 if t - t_rotation < 8 else 0.5)
            elif ias < vr - 8 or vs > 2000:
                profondeur = max(profondeur - 0.3 * dt, 0.03)
            elif vs < 1000:
                profondeur = min(profondeur + 0.1 * dt, 0.45)
            sim.envoyer("ELEVATOR_SET", s_prof * profondeur * 16383)
        if not au_sol and (t_train is not None or (agl > 30 and vs > 300)):
            t_train = 1.0 if t_train is None else t_train + dt
            if t_train >= 1:                              # tout de suite, puis une fois par seconde
                t_train = 0.0
                train.pas(sim)

        # A 150 ft sol, le pilote automatique prend le relais : axe de piste et montee.
        if not au_sol and agl >= 150:
            if t_ap == 0 or t_ap % 1 < dt:                # une demande par seconde
                sim.envoyer("AUTOPILOT_ON")
            t_ap += dt
            if booleen(sim.lire_vite("AUTOPILOT_MASTER")) is True:
                sim.envoyer("ELEVATOR_SET", 0)            # le pilote automatique tient l'avion
                sim.envoyer("RUDDER_SET", 0)
                mode_cap(sim, cap_piste)
                prendre_croisiere_du_controle(sim, a)
                engager_vertical(sim, a.croisiere, a.vs_montee)
                dire(f"Avion en l'air{', train rentre' if train.rentre else ''}, "
                     f"pilote automatique engage : axe {cap_piste:03d}, "
                     f"montee vers {a.croisiere} ft a {a.vs_montee} ft/min.", tel=True)
                return True
            if t_ap >= 10:
                dire("Le pilote automatique ne s'engage pas : REPRENDS LA MAIN.", alerte=True)


# --------------------------------------------------------------------------
# Approche et atterrissage automatiques
# --------------------------------------------------------------------------
def mesurer_sens(sim, a):
    """A l'arret au sol : sens des trois gouvernes et hauteur des capteurs au-dessus du sol.
    Sans eux, pas d'arrondi possible (on ne peut pas tater les gouvernes en vol). Une fois
    mesures, ils sont retenus pour cet avion : plus besoin de les remesurer."""
    a.sens = {"pal": sens_commande(sim, "RUDDER_SET", "RUDDER_RIGHT", "RUDDER_POSITION"),
              "prof": sens_commande(sim, "ELEVATOR_SET", "ELEV_UP", "ELEVATOR_POSITION"),
              "aile": sens_commande(sim, "AILERON_SET", "AILERONS_RIGHT", "AILERON_POSITION")}
    h = num(frais(sim, "PLANE_ALT_ABOVE_GROUND"), None)
    if h is not None and 0 <= h < 30:
        a.hauteur_sol = h
    if None in a.sens.values():
        dire("Je n'arrive pas a lire toutes les gouvernes de cet avion : je ferai l'approche, mais je "
             "te rendrai l'avion a 3 NM de la piste pour l'atterrissage.", tel=True)
    else:
        retenir_sens(sim, a)


def retenir_sens(sim, a):
    """Garde le sens des gouvernes de cet avion (par son nom dans le simulateur) dans les reglages."""
    titre = texte_sim(sim.lire("TITLE"))
    if not titre or not all(a.sens.get(k) for k in ("pal", "prof", "aile")):
        return
    avions = lire_reglages().get("avions")
    avions = avions if isinstance(avions, dict) else {}
    avions[titre] = dict(a.sens, sol=a.hauteur_sol)
    garder_reglages(avions=avions)


def sens_retenus(sim, a):
    """Reprend le sens des gouvernes deja mesure pour cet avion. Renvoie True s'il etait connu."""
    avions = lire_reglages().get("avions")
    connu = avions.get(texte_sim(sim.lire("TITLE"))) if isinstance(avions, dict) else None
    if not isinstance(connu, dict) or not all(connu.get(k) in (1, -1) for k in ("pal", "prof", "aile")):
        return False
    a.sens = {k: connu[k] for k in ("pal", "prof", "aile", "incl") if connu.get(k) in (1, -1)}
    a.hauteur_sol = num(connu.get("sol"), 3.0)
    return True


def TrainSorti():
    def encore_rentre(sim):
        v = booleen(sim.lire_vite("GEAR_HANDLE_POSITION"))
        return None if v is None else not v

    return Rentree(
        "Train", "sorti", encore_rentre,
        [lambda sim: sim.envoyer("GEAR_DOWN"),
         lambda sim: sim.envoyer("GEAR_SET", 1),
         lambda sim: sim.variable("GEAR HANDLE POSITION", "Bool", 1),
         lambda sim: sim.envoyer("GEAR_TOGGLE")],          # la manette est lue rentree : la bascule la sort
        fixe=lambda sim: booleen(sim.lire("IS_GEAR_RETRACTABLE")) is False)


def preparer_approche(sim, a, avion):
    """Choisit la piste et la vitesse d'approche. Renvoie (etat de l'approche, None), ou
    (None, raison) si l'approche automatique est impossible."""
    if not a.code_arrivee:
        return None, "je ne connais pas le code de l'aeroport d'arrivee (donne-le : vol 9000 LFMD)"
    vent_de, vent_kt = num(sim.lire("AMBIENT_WIND_DIRECTION")), num(sim.lire("AMBIENT_WIND_VELOCITY"))
    try:
        piste = choisir_piste(a.code_arrivee, vent_de, vent_kt, a.piste)
    except Exception:
        return None, "je n'ai pas pu telecharger la liste des pistes (connexion internet ?)"
    if piste is None:
        return None, (f"je ne trouve pas la piste {a.piste} a {a.code_arrivee}" if a.piste
                      else f"je ne trouve aucune piste utilisable a {a.code_arrivee}")
    if piste["alt_ft"] is None:
        piste["alt_ft"] = a.alt_terrain or 0.0
    vapp = vitesse_approche(avion)
    if not vapp:
        return None, "je ne connais pas la vitesse de decrochage de cet avion"
    pose = arrondi_possible(a)
    dire(f"Approche : piste {piste['ident']} de {a.code_arrivee} (axe {piste['cap']:.0f} vrai, "
         f"{piste['longueur_m']:.0f} m, vent du {vent_de:.0f} a {vent_kt:.0f} kt), vitesse d'approche {vapp} kt. "
         + ("Je me poserai et je m'arreterai sur la piste." if pose
            else "Je te rendrai l'avion a 3 NM de la piste, stabilise, pour l'atterrissage."), tel=True)
    return nouvelle_approche(piste, vapp, pose), None


def vitesse_approche(avion):
    """1,3 fois la vitesse de decrochage (volets sortis si connue), plus 5 kt ; None si inconnue."""
    decrochage = avion["vs0"] or avion["vs1"]
    return round(1.3 * decrochage + 5) if decrochage else None


def arrondi_possible(a):
    """L'arrondi demande la main sur les gaz et le sens des trois gouvernes."""
    return bool(a.gaz) and all(a.sens.get(k) for k in ("pal", "prof", "aile"))


def nouvelle_approche(piste, vapp, pose):
    return {"piste": piste, "vapp": vapp, "pose": pose, "etabli": False, "plan": False, "stable": False,
            "hist": [], "wca": 0.0, "decl": 0.0, "bug": None, "vs_cmd": None, "t_volets": 0,
            "train": TrainSorti(), "ils": "non" if piste.get("fictive") else None, "ils_t": 0.0,
            "cap_prec": None, "raison": ""}


def preparer_repetition(sim, a, avion):
    """Piste imaginaire 2000 ft sous l'avion, 8 NM devant lui dans l'axe de son cap : le bot y
    fait toute l'approche et l'arrondi en plein ciel, pour regler l'arrondi sans risquer l'avion.
    Renvoie (etat de l'approche, None) ou (None, raison)."""
    if not arrondi_possible(a):
        return None, ("je ne connais pas le sens des gouvernes de cet avion (envoie 'vol' une fois avec "
                      "l'avion a l'arret au sol)" if a.gaz else "tu m'as interdit de toucher aux gaz (--sans-gaz)")
    vapp = vitesse_approche(avion)
    if not vapp:
        return None, "je ne connais pas la vitesse de decrochage de cet avion"
    if num(sim.lire("PLANE_ALT_ABOVE_GROUND")) < 3500:
        return None, "il faut au moins 3500 ft de vide sous l'avion ; monte d'abord (altitude ...)"
    pos, cap = position(sim), num(sim.lire("PLANE_HEADING_DEGREES_TRUE"), None)
    if pos is None or cap is None:
        return None, "je ne lis pas la position de l'avion"
    cap = math.degrees(cap) % 360
    d = 8 * NM / 6371000                                     # 8 NM devant, en radians
    lat = pos[0] + math.degrees(d * math.cos(math.radians(cap)))
    lon = pos[1] + math.degrees(d * math.sin(math.radians(cap)) / math.cos(math.radians(pos[0])))
    piste = {"ident": "imaginaire", "lat": lat, "lon": lon, "cap": cap, "longueur_m": 1500.0,
             "largeur_m": 30.0, "visee_m": 300.0, "fictive": True,
             "alt_ft": num(sim.lire("PLANE_ALTITUDE")) - 2000}
    dire(f"Repetition : piste imaginaire 2000 ft sous l'avion, 8 NM devant, axe {cap:.0f}. Je fais "
         f"l'approche a {vapp} kt et l'arrondi, puis je remonte et je reprends le vol.", tel=True)
    return nouvelle_approche(piste, vapp, True), None


def approche_pas(sim, a, app, vitesse):
    """Une seconde d'approche : rejoint le point a 10 NM dans l'axe, suit l'axe et un plan a 3 degres
    (ou laisse le mode APR de l'avion suivre l'ILS), sort train et volets, et verifie que l'approche
    est stabilisee. Renvoie None, 'arrondi', 'main' (rendre l'avion) ou 'remise' (raison dans app)."""
    p = app["piste"]
    pos = position(sim)
    hv, hm = num(sim.lire("PLANE_HEADING_DEGREES_TRUE"), None), num(sim.lire("PLANE_HEADING_DEGREES_MAGNETIC"), None)
    if pos is None or hv is None or hm is None:
        return None
    hv, hm = math.degrees(hv) % 360, math.degrees(hm) % 360
    app["decl"] = angle(hv - hm)
    t = sim.horloge()
    avant, ecart = sur_piste(p, *pos)
    alt = num(sim.lire("PLANE_ALTITUDE"))                     # altitude vraie, comme celle de la piste
    agl = num(sim.lire("PLANE_ALT_ABOVE_GROUND"), 9999) - a.hauteur_sol
    if p.get("fictive"):
        if agl < 1000:                                        # repetition : le vrai sol reste loin
            app["raison"] = "vrai relief a moins de 1000 ft"
            return "remise"
        agl = alt - p["alt_ft"]                               # hauteur au-dessus de la piste imaginaire
    ias = num(sim.lire("AIRSPEED_INDICATED"))
    gs_kt = num(sim.lire("GPS_GROUND_SPEED")) * 1.944
    plan = p["alt_ft"] + (avant + p["visee_m"]) * math.tan(math.radians(3)) / 0.3048

    # Derive due au vent : ecart entre le cap de l'avion et sa route reelle (mesuree sur 2 a 4 s).
    app["hist"] = [h for h in app["hist"] if t - h[0] <= 4] + [(t, avant, ecart)]
    t0, av0, ec0 = app["hist"][0]
    if t - t0 >= 2 and math.hypot(avant - av0, ecart - ec0) > 30:
        route = p["cap"] + math.degrees(math.atan2(ecart - ec0, av0 - avant))
        app["wca"] += (max(-25.0, min(25.0, angle(hv - route))) - app["wca"]) * 0.3
    # Sens de l'inclinaison (le simulateur ne le dit pas) : en virage, l'avion penche du cote ou il tourne.
    if app["cap_prec"]:
        virage = angle(hv - app["cap_prec"][1]) / max(t - app["cap_prec"][0], 0.5)
        incl = math.degrees(num(sim.lire("PLANE_BANK_DEGREES")))
        if abs(incl) > 8 and abs(virage) > 1 and a.sens.get("incl") != (1 if incl * virage > 0 else -1):
            a.sens["incl"] = 1 if incl * virage > 0 else -1
            retenir_sens(sim, a)
    app["cap_prec"] = (t, hv)

    if not app["etabli"]:
        dans_axe = 1.5 * NM < avant < 12 * NM and abs(ecart) < 2 * NM and abs(angle(hv - p["cap"])) < 100
        if dans_axe or math.hypot(avant - 10 * NM, ecart) < NM:
            app["etabli"] = True
            dire(f"Je rejoins l'axe de la piste {p['ident']} a {avant / NM:.0f} NM du seuil.", tel=True)

    # ILS capte a la radio et aligne sur la piste : on laisse le mode APR de l'avion le suivre.
    if app["ils"] is None and app["etabli"] and avant < 12 * NM and abs(ecart) < NM:
        cours = num(sim.lire("NAV_LOCALIZER:1"), None)
        if cours is not None and abs(cours) <= 6.4:
            cours = math.degrees(cours)                    # la bibliotheque renvoie parfois des radians
        ok = (booleen(sim.lire("NAV_HAS_LOCALIZER:1")) and booleen(sim.lire("NAV_HAS_GLIDE_SLOPE:1"))
              and cours is not None and min(abs(angle(cours - p["cap"])), abs(angle(cours - p["cap"] + app["decl"]))) < 10)
        app["ils"], app["ils_t"] = ("arme" if ok and sim.envoyer("AP_APR_HOLD_ON") else "non"), t
        if app["ils"] == "arme":
            dire("ILS de la piste capte a la radio : j'arme le mode APR de l'avion.", tel=True)
    apr = app["ils"] == "arme" and booleen(sim.lire("AUTOPILOT_APPROACH_HOLD")) is True
    glide = apr and booleen(sim.lire("AUTOPILOT_GLIDESLOPE_HOLD")) is True
    if app["ils"] == "arme" and not apr and t - app["ils_t"] > 60:
        app["ils"], app["bug"] = "non", None
        sim.envoyer("AP_APR_HOLD_OFF")
        mode_cap(sim, round(hm) % 360)
        dire("Le mode APR ne s'engage pas : je guide l'approche moi-meme.", tel=True)

    # Lateral : vers le point a 10 NM dans l'axe, puis sur l'axe (30 degres d'interception au plus).
    if not apr:
        if app["etabli"] and avant < 3 * NM:              # courte finale : on serre l'axe
            route_voulue = p["cap"] - max(-10.0, min(10.0, ecart / 12))
        elif app["etabli"]:
            route_voulue = p["cap"] - max(-30.0, min(30.0, ecart / 30))
        else:
            route_voulue = p["cap"] + math.degrees(math.atan2(-ecart, avant - 10 * NM))
        bug = round(route_voulue + app["wca"] - app["decl"]) % 360
        if bug != app["bug"]:
            sim.envoyer("HEADING_BUG_SET", bug)
            app["bug"] = bug

    # Vertical : palier jusqu'a rejoindre le plan a 3 degres, puis taux = vitesse sol x 5,3 corrige.
    if not app["plan"] and (glide or (app["etabli"] and avant < 12 * NM and alt >= plan - 50)):
        app["plan"] = True
        dire(f"Sur le plan de descente, a {avant / NM:.1f} NM du seuil.", tel=True)
        if not glide:
            # Altitude affichee sous la piste : le pilote automatique ne doit jamais se mettre en palier.
            engager_vertical(sim, max(int((p["alt_ft"] - 300) // 100 * 100), 0), -500)
            app["vs_cmd"] = -500
    if app["plan"] and not glide:
        vs = round(max(-1800, min(-100, -gs_kt * 5.3 - 3 * (alt - plan))), -2)
        if abs(vs - app["vs_cmd"]) >= 100:
            app["vs_cmd"] = vs
            regler_vs(sim, vs)

    # Vitesse et configuration : train a 6 NM ou sur le plan, volets un cran toutes les 6 s.
    cible = app["vapp"] + (0 if app["plan"] else 20)
    if vitesse:
        vitesse["cible"] = cible
    else:
        sim.envoyer("AP_SPD_VAR_SET", cible)              # automanette eventuelle
    if app["plan"] or (app["etabli"] and avant < 6 * NM):
        app["train"].pas(sim)
    app["t_volets"] += 1
    if app["etabli"] and app["t_volets"] >= 6 and ias < app["vapp"] + 35:
        app["t_volets"] = 0
        if num(sim.lire("FLAPS_HANDLE_INDEX")) < num(sim.lire("FLAPS_NUM_HANDLE_POSITIONS"), 2):
            sim.envoyer("FLAPS_INCR")

    # Verifications
    if not app["pose"] and app["etabli"] and avant <= 3 * NM:
        return "main"
    if app["etabli"] and avant > 2 * NM and agl < 400:
        app["raison"] = "trop pres du relief"
        return "remise"
    if avant < NM and not app["stable"]:                  # sur le plan, on passe 500 ft vers 1,4 NM
        app["raison"] = "pas stabilise a 1 NM du seuil"
        return "remise"
    if app["plan"] and agl <= 500:
        defauts = []
        if abs(ecart) > (90 if app["stable"] else 60):
            defauts.append(f"{ecart:+.0f} m de l'axe")
        if abs(alt - plan) > (200 if app["stable"] else 150):
            defauts.append(f"{alt - plan:+.0f} ft par rapport au plan")
        if not app["stable"] and not app["vapp"] - 5 <= ias <= app["vapp"] + 20:
            defauts.append(f"{ias:.0f} kt au lieu de {app['vapp']}")
        if booleen(sim.lire("GEAR_HANDLE_POSITION")) is False:
            defauts.append("train rentre")
        if defauts:
            app["raison"] = "approche non stabilisee : " + ", ".join(defauts)
            return "remise"
        if not app["stable"]:
            app["stable"] = True
            dire("500 ft : approche stabilisee, je continue.", tel=True)
        if agl <= 150:
            return "arrondi"
    return None


def remettre_gaz(sim, a, app, raison):
    """Remise de gaz : plein gaz, pilote automatique, montee dans l'axe. Le pilote reprend la main,
    sauf en repetition (piste imaginaire) : le bot remonte a la croisiere et reprend le vol."""
    p = app["piste"]
    for axe in ("ELEVATOR_SET", "AILERON_SET", "RUDDER_SET"):   # on lache les gouvernes tenues a la main
        sim.envoyer(axe, 0)
    sim.envoyer("THROTTLE_SET", 16383)
    if booleen(frais(sim, "AUTOPILOT_MASTER")) is not True:
        sim.envoyer("AUTOPILOT_ON")
    if app["ils"] == "arme":
        sim.envoyer("AP_APR_HOLD_OFF")
    mode_cap(sim, round(p["cap"] - app["decl"]) % 360)
    if p.get("fictive"):
        engager_vertical(sim, a.croisiere, 800)
        if raison:
            dire(f"Repetition interrompue ({raison}) : plein gaz, je remonte a {a.croisiere} ft.", alerte=True)
        return
    a.approche = max(a.approche or 0, int(math.ceil((p["alt_ft"] + 1500) / 100) * 100))
    engager_vertical(sim, a.approche, 800)
    dire(f"REMISE DE GAZ ({raison}) : plein gaz, je remonte a {a.approche} ft dans l'axe de la piste "
         f"{p['ident']}. Rentre le train et les volets, A TOI.", alerte=True)


ENTETE_ARRONDI = ("t;vitesse_kt;hauteur_ft;vs_ft_min;inclinaison;ecart_axe_m;ecart_cap_deg;profondeur;"
                  "aileron;palonnier;gaz;au_sol")


def arrondir_et_rouler(sim, a, app, vitesse):
    """De 150 ft jusqu'a l'arret sur la piste. Renvoie 'pose' ou 'remise'."""
    mesures = []
    try:
        return _arrondi(sim, a, app, vitesse, mesures)
    finally:
        enregistrer_mesures(mesures, "atterrissage", ENTETE_ARRONDI)


def _arrondi(sim, a, app, vitesse, mesures):
    p, s = app["piste"], a.sens
    s_incl = s.get("incl", -1)
    t0 = t_prec = sim.horloge()
    t_man = t_sol = None              # coupure du pilote automatique, toucher
    t_gaz, remise, raison = 0.0, False, ""
    gaz = vitesse["gaz"] if vitesse else num(sim.lire("GENERAL_ENG_THROTTLE_LEVER_POSITION:1"), 30) / 100
    gaz_debut = gaz
    # Repetition : la hauteur se compte au-dessus de la piste imaginaire, et le vrai sol est surveille.
    fictive = p.get("fictive", False)
    v = {"agl": 150.0 + (p["alt_ft"] if fictive else a.hauteur_sol), "sol": False, "ias": app["vapp"],
         "gs": app["vapp"], "pos": None, "reel": 9999.0}
    cap, incl, vs = p["cap"], 0.0, -500.0
    cap_prec, incl_prec, taux_cap, taux_incl = None, 0.0, 0.0, 0.0
    profondeur = aileron = palonnier = 0.0
    vs_toucher = ecart_toucher = vs_air = 0.0
    lectures = {"agl": "PLANE_ALTITUDE" if fictive else "PLANE_ALT_ABOVE_GROUND", "ias": "AIRSPEED_INDICATED",
                "gs": "GROUND_VELOCITY", "pos": None}
    lectures.update({"reel": "PLANE_ALT_ABOVE_GROUND"} if fictive else {"sol": "SIM_ON_GROUND"})
    n = 0
    while True:
        sim.attendre(0.02)
        maintenant = sim.horloge()
        dt, t_prec, t = max(maintenant - t_prec, 0.005), maintenant, maintenant - t0
        # Inclinaison, vitesse verticale et cap a chaque tour, le reste a tour de role.
        for nom_sim, quoi in (("PLANE_BANK_DEGREES", "incl"), ("VERTICAL_SPEED", "vs"),
                              ("PLANE_HEADING_DEGREES_TRUE", "cap")):
            x = sim.lire_vite(nom_sim)
            if x is not None:
                if quoi == "incl":
                    incl = s_incl * math.degrees(num(x))       # positif : penche a droite
                elif quoi == "vs":
                    vs = num(x)
                else:
                    cap = math.degrees(num(x)) % 360
        nom = list(lectures)[n % len(lectures)]
        n += 1
        x = position(sim, vite=True) if nom == "pos" else sim.lire_vite(lectures[nom])
        if nom == "sol":
            x = booleen(x)
        if x is not None:
            v[nom] = x if nom in ("pos", "sol") else num(x)
        agl = v["agl"] - (p["alt_ft"] if fictive else a.hauteur_sol)
        if fictive:
            v["sol"] = agl <= 0
            if v["reel"] < 800 and not remise:
                remise, raison = True, "vrai relief a moins de 800 ft"
        ecart = sur_piste(p, *v["pos"])[1] if v["pos"] else 0.0
        ecart_cap = angle(p["cap"] - cap)                     # positif : il faut tourner a droite
        if cap_prec is not None:
            taux_cap += (angle(cap - cap_prec) / dt - taux_cap) * min(dt / 0.5, 1.0)
        taux_incl += ((incl - incl_prec) / dt - taux_incl) * min(dt / 0.3, 1.0)
        cap_prec, incl_prec = cap, incl

        if t_man is None:
            # Le pilote automatique tient encore l'avion ; on garde la vitesse jusqu'a 50 ft.
            if vitesse and t - t_gaz >= 1:
                t_gaz = t
                tenir_vitesse(sim, vitesse, v["ias"])
                gaz = gaz_debut = vitesse["gaz"]
            if agl > 50 and not v["sol"]:
                if remise or abs(ecart) > 90 or abs(incl) > 20 or t > 90:
                    remettre_gaz(sim, a, app, raison if remise else
                                 f"{ecart:+.0f} m de l'axe en courte finale" if abs(ecart) > 90
                                 else "inclinaison anormale" if abs(incl) > 20 else "l'avion ne descend plus")
                    return "remise"
                continue
            t_man, vs_approche = t, min(vs, -200)
            sim.envoyer("AUTOPILOT_OFF")
            if booleen(frais(sim, "AUTOPILOT_MASTER")) is True:
                sim.envoyer("AP_MASTER")
            if booleen(sim.lire("AUTOPILOT_THROTTLE_ARM")) is True:
                sim.envoyer("AUTO_THROTTLE_ARM")
            dire("50 ft : pilote automatique coupe, arrondi.")

        # Gaz : reduits en 3 s, au ralenti des le toucher ; plein gaz en remise.
        gaz = 1.0 if remise else 0.0 if t_sol is not None else max(gaz_debut * (1 - (t - t_man) / 3), 0.0)
        sim.envoyer("THROTTLE_SET", gaz * 16383)
        if not v["sol"] and t_sol is None:
            vs_air = min(vs_air * 0.9, vs) if vs < 0 else vs_air * 0.9   # taux de chute juste avant le toucher
        if fictive and v["sol"] and not remise:
            # « Toucher » sur la piste imaginaire : bilan, puis le bot remonte reprendre le vol.
            app["bilan"] = (vs_air, ecart, v["ias"], t - t_man)
            mesures.append((round(t, 2), round(v["ias"]), round(agl), round(vs), round(incl, 1), round(ecart, 1),
                            round(ecart_cap, 1), round(profondeur, 3), round(aileron, 3), round(palonnier),
                            round(gaz, 2), 1))
            remettre_gaz(sim, a, app, None)
            return "repetition"
        if v["sol"] and not remise:
            if t_sol is None:
                t_sol, vs_toucher, ecart_toucher = t, vs_air, ecart
                sim.envoyer("SPOILERS_ON")
            profondeur = max(profondeur - 0.15 * dt, 0.0)         # on repose la roulette de nez
            aileron = 0.0
            palonnier = loi_palonnier(ecart_cap - max(-5.0, min(5.0, ecart / 3)), taux_cap, v["ias"])
            if t - t_sol > 1.5 and v["gs"] > 3:
                sim.envoyer("BRAKES")
        else:
            # Arrondi : on garde le taux de l'approche jusqu'a ce qu'il depasse -(150 + 10 x hauteur),
            # soit vers 25 ft, puis il diminue avec la hauteur jusqu'a -150 ft/min au toucher (environ 6 s).
            cible = 700 if remise else max(-(150 + 10 * agl), vs_approche)
            profondeur = max(-0.3, min(0.7, profondeur + 0.0005 * (cible - vs) * dt))
            incl_voulue = 0.0 if remise else max(-3.0, min(3.0, -ecart / 5))
            aileron = max(-0.4, min(0.4, 0.04 * (incl_voulue - incl) - 0.02 * taux_incl))
            # Sous 15 ft, le palonnier aligne le nez sur la piste (l'avion arrive en crabe par vent de travers).
            palonnier = max(-6000.0, min(6000.0, 300 * ecart_cap - 300 * taux_cap)) if agl < 15 and not remise else 0.0
        sim.envoyer("ELEVATOR_SET", s["prof"] * profondeur * 16383)
        sim.envoyer("AILERON_SET", s["aile"] * aileron * 16383)
        sim.envoyer("RUDDER_SET", s["pal"] * palonnier)
        mesures.append((round(t, 2), round(v["ias"]), round(agl), round(vs), round(incl, 1), round(ecart, 1),
                        round(ecart_cap, 1), round(profondeur, 3), round(aileron, 3), round(palonnier),
                        round(gaz, 2), int(bool(v["sol"]))))

        if not remise:
            if t_sol is None and (abs(incl) > 20 or (vs < -1300 and agl < 40) or t - t_man > 25):
                remise, raison = True, ("inclinaison anormale" if abs(incl) > 20 else
                                        "chute trop rapide" if vs < -1300 else "l'avion ne se pose pas")
            elif t_sol is not None and not v["sol"] and agl > 15:
                remise, raison = True, "rebond"
        if remise and agl > 300 and vs > 200:
            remettre_gaz(sim, a, app, raison)
            return "remise"
        if t_sol is not None and not remise and ((v["sol"] and v["gs"] < 3) or t - t_sol > 120):
            for axe in ("ELEVATOR_SET", "AILERON_SET", "RUDDER_SET", "THROTTLE_SET"):
                sim.envoyer(axe, 0)
            if booleen(frais(sim, "BRAKE_PARKING_POSITION")) is False:
                sim.envoyer("PARKING_BRAKES")
            sim.envoyer("FLAPS_UP")
            pos = position(sim, vite=True)
            app["bilan"] = (vs_toucher, ecart_toucher, -sur_piste(p, *pos)[0] if pos else None)
            return "pose"


# --------------------------------------------------------------------------
# Vol
# --------------------------------------------------------------------------
def voler(sim, a, ordres):
    ordres["en_vol"] = True
    try:
        _voler(sim, a, ordres)
    finally:
        ordres.clear()


def _voler(sim, a, ordres):
    sim = Tampon(sim)
    for nom, defaut in (("sens", {}), ("hauteur_sol", 3.0), ("atterrissage", False), ("code_arrivee", None),
                        ("alt_terrain", None), ("piste", None)):
        vars(a).setdefault(nom, defaut)
    route = f"cap {a.cap:03d} garde tel quel" if a.cap is not None else "en suivant le plan de vol"
    terrain = getattr(a, "terrain", None)
    remise = (f"arrivee {terrain}, je te rends l'avion {HAUTEUR_REMISE_M} m au-dessus, a {a.approche} ft"
              if terrain else f"remise a {a.approche} ft" if a.approche is not None
              else "arrivee a lire dans le plan de vol")
    recherche = {"t": 0, "echec": False}

    def trouver_arrivee():
        """Tant que l'arrivee est inconnue : la cherche dans le plan de vol, ou prend celle
        donnee par 'arrivee LFMD'. Dit sur Telegram ce qui a ete trouve."""
        code = ordres.pop("arrivee", None)
        try:
            if code:
                ap = chercher_aeroport(code)
                TG.envoyer(fixer_arrivee(a, ap, "donnee par toi") if ap
                           else f"Je ne connais pas l'aeroport {code}. Verifie le code.")
                return
            if a.approche is not None:
                return
            recherche["t"] -= 1
            if recherche["t"] > 0:
                return
            recherche["t"] = 30
            ap = arrivee_du_plan(sim)
            if ap:
                dire(fixer_arrivee(a, ap, "lue dans le plan de vol"), tel=True)
        except Exception:
            if not recherche["echec"]:
                recherche["echec"] = True
                dire("Je n'ai pas pu telecharger la liste des aeroports (connexion internet ?). "
                     "Envoie 'stop' puis donne l'altitude toi-meme : vol 9000 3300", alerte=True)
    croisiere = (f"croisiere donnee par le controle apres le decollage (sinon {a.croisiere} ft, comme au "
                 "dernier vol)" if getattr(a, "suivre_controle", False) else f"croisiere {a.croisiere} ft")
    dire(f"Vol pret : {croisiere}, {remise}, {route}, "
         f"{'taux de montee regle automatiquement' if a.taux_auto else f'montee a {a.vs_montee} ft/min'}. "
         "Je prends l'avion a 1000 ft sol.", tel=True)
    avion = fiche_avion(sim)
    if "HELICO" in avion["categorie"]:
        dire("Cet appareil est un helicoptere : je ne sais piloter que des avions. Vol annule.", tel=True)
        return
    if avion["ap"] is False:
        dire("Cet avion n'a pas de pilote automatique : je ne peux pas le piloter. Vol annule.", tel=True)
        return
    dire("Avion : " + ", ".join(
        [f"cabre a {avion['vr']} kt" if avion["vr"] else f"vitesse de cabre inconnue ({a.vr} kt par defaut)",
         f"montee a {avion['montee']:.0f} kt" if avion["montee"] else "vitesse de montee inconnue",
         f"{avion['moteurs']} moteur(s)"]) + ".")
    suivi_min = lire_reglages().get("suivi")
    suivi_min = suivi_min if isinstance(suivi_min, int) else 0
    trouver_arrivee()
    # Avion au parking (a l'arret, frein de parking serre ou moteur arrete) : on le prepare d'office.
    if (booleen(sim.lire("SIM_ON_GROUND")) is True and num(frais(sim, "GROUND_VELOCITY"), 99) < 2
            and (booleen(frais(sim, "BRAKE_PARKING_POSITION")) is True
                 or moteurs_en_marche(sim) is False)):
        preparer(sim)
    decollage_auto = volets_a_rentrer = False
    train, volets, volets_go = Train(), Volets(), False
    vr = a.vr
    sens_mesure = bool(a.sens) or sens_retenus(sim, a)
    if sens_mesure and a.atterrissage:
        dire("Je connais deja les gouvernes de cet avion : je pourrai me poser moi-meme.")
    while sim.lire("SIM_ON_GROUND") or num(sim.lire("PLANE_ALT_ABOVE_GROUND")) < 1000:
        # Avion inconnu : sens des gouvernes mesure (une seule fois par avion) tant qu'il est a l'arret.
        if (not sens_mesure and booleen(sim.lire("SIM_ON_GROUND")) is True
                and num(frais(sim, "GROUND_VELOCITY"), 99) < 2):
            sens_mesure = True
            mesurer_sens(sim, a)
        traiter_commandes(sim, lambda: "Vol pret : j'attends que tu decolles et passes 1000 ft sol."
                          if not decollage_auto else "Montee initiale apres le decollage automatique.", ordres)
        if ordres.pop("stop", None):
            dire("Vol annule." if not decollage_auto else "Je lache l'avion. A toi.", tel=True)
            return
        trouver_arrivee()
        if ordres.pop("prepare", None):
            preparer(sim)
        if ordres.pop("moteur", None):
            demarrer_moteur(sim)
        demande = ordres.pop("decollage", None)
        if demande and not decollage_auto:
            vr = demande if demande is not True else avion["vr"] or a.vr
            decollage_auto = volets_a_rentrer = decoller(sim, a, ordres, vr, train)
        if decollage_auto:
            train.pas(sim)                   # on insiste tant que le train n'est pas confirme rentre
        # Apres un decollage automatique : si le controle donne (ou change) l'altitude, on la prend.
        if decollage_auto and prendre_croisiere_du_controle(sim, a):
            engager_vertical(sim, a.croisiere, a.vs_montee)
        # Apres un decollage automatique : volets rentres des que la vitesse le permet.
        if volets_a_rentrer and num(sim.lire("AIRSPEED_INDICATED")) >= vr + 25 \
                and num(sim.lire("PLANE_ALT_ABOVE_GROUND")) >= 400:
            volets_a_rentrer, volets_go = False, True
        if volets_go:
            volets.pas(sim)                  # on insiste tant que les volets ne sont pas confirmes rentres
        sim.attendre(1)

    # --- Prise en main et montee ---
    sim.envoyer("AUTOPILOT_ON")
    train.pas(sim)
    volets.pas(sim)
    if a.cap is not None:
        mode_cap(sim, a.cap)
        lateral = "FIXE"
        dire(f"Lateral : cap {a.cap:03d} demande au depart, garde tel quel.")
    else:
        lateral = engager_lateral(sim, a)
    cap_envoye, t_cap, cap_refuse = None, 0, 0
    # Si tu es deja au-dessus de l'altitude de croisiere, il faut descendre vers elle.
    alt_depart = altitude(sim)
    prendre_croisiere_du_controle(sim, a)
    taux = a.vs_montee if a.croisiere >= alt_depart else -max(a.vs_montee, 500)
    engager_vertical(sim, a.croisiere, taux)
    dire(f"J'ai l'avion. {'Montee' if taux > 0 else 'Descente'} vers {a.croisiere} ft"
         + (", taux regle sur la vitesse de l'avion." if a.taux_auto and taux > 0 else f" a {taux} ft/min."), tel=True)
    sim.envoyer("BAROMETRIC")          # altimetre cale sur la pression du moment
    # Montee automatique : on vise la vitesse de montee de l'avion (a defaut celle du moment).
    v_montee = avion["montee"] or max(num(sim.lire("AIRSPEED_INDICATED")), 1.4 * (avion["vs1"] or 0))
    v_plancher = 1.25 * avion["vs1"] if avion["vs1"] else 0.8 * v_montee
    t_taux, ias_avant = 0, None
    t_baro, t_suivi, t_carburant, phares, relief_signale = 0, 0, 0, None, False

    phase = "MONTEE"
    vitesse = None                      # etat du correcteur de gaz, actif en descente
    echecs, t_verif, t_log = 0, 0, 0
    t_controle, controle_prec = 0, None
    sans_distance_signale = False
    mesures_prec = None
    infos = {}
    ap_coupe = 0
    vs_cmd, t_vs, trop_haut_signale = -1500, 0, False
    dist_min = None
    prevenu_10_min = False
    app = None                          # etat de l'approche automatique (ou de la repetition)
    cap_repetition = gaz_a_rendre = None   # a reprendre apres une repetition
    noms = {"MONTEE": "En montee", "CROISIERE": "En croisiere", "DESCENTE": "En descente",
            "APPROCHE": "En approche",
            "REMISE": "Arrivee proche, j'attends que tu reprennes la main"}

    def texte_etat():
        if not infos:
            return "Je viens de prendre l'avion."
        d = f"Arrivee a {infos['dist']:.0f} NM" if infos["dist"] is not None else "Distance a l'arrivee inconnue"
        return (f"{noms[infos['phase']]}{' (simulateur en pause)' if infos['fige'] else ''}\n"
                f"{infos['alt']:.0f} ft, {infos['ias']:.0f} kt, {infos['vs']:+.0f} ft/min\n"
                f"{d}\n{estimation(sim, infos['dist'], a.remise).capitalize()}"
                + (f"\nCarburant : {int(infos['autonomie'])} h {int(infos['autonomie'] % 1 * 60):02d} d'autonomie"
                   if infos.get("autonomie") else "")
                + ("\nCap fixe (envoie 'route' pour reprendre le plan de vol)" if lateral == "FIXE" else ""))
    sim.attendre(5)

    while True:
        sim.attendre(1)
        t_verif += 1
        t_log += 1
        alt = altitude(sim)
        ias = num(sim.lire("AIRSPEED_INDICATED"))
        vs = num(sim.lire("VERTICAL_SPEED"))
        dist = distance_restante(sim, a.dest)
        # Simulateur en pause : toutes les mesures sont identiques a la seconde d'avant.
        fige = (alt, ias, dist) == mesures_prec
        mesures_prec = (alt, ias, dist)
        infos = {"phase": phase, "alt": alt, "ias": ias, "vs": vs, "dist": dist, "fige": fige,
                 "autonomie": infos.get("autonomie")}
        if phase in ("MONTEE", "CROISIERE"):
            trouver_arrivee()
        elif ordres.pop("arrivee", None):
            TG.envoyer("Trop tard pour changer d'arrivee : je suis deja en descente.")
        # 3 NM par 1000 ft a perdre, plus une marge. Inconnu tant que l'arrivee n'est pas trouvee.
        tod = None if a.approche is None else a.remise + max(a.croisiere - a.approche, 0) / 1000 * 3 + 10

        # Ordres recus par Telegram
        traiter_commandes(sim, texte_etat, ordres)
        if ordres.pop("stop", None):
            dire("Je lache l'avion : le pilote automatique reste dans son etat actuel. A toi.", tel=True)
            return
        if "cap" in ordres:
            cap_fixe = ordres.pop("cap")
            mode_cap(sim, cap_fixe)
            lateral, cap_envoye, cap_refuse = "FIXE", None, 0
            dire(f"Cap {cap_fixe:03d} regle. Je le garde tel quel ; envoie 'route' pour "
                 "reprendre le plan de vol.", tel=True)
        if ordres.pop("route", None):
            lateral = engager_lateral(sim, a)
            cap_envoye, cap_refuse = None, 0
            TG.envoyer("Je reprends le plan de vol." if lateral != "FIXE"
                       else "Pas de plan de vol actif dans le GPS : je garde le cap actuel.")
        if any([ordres.pop("prepare", None), ordres.pop("decollage", None), ordres.pop("moteur", None)]):
            TG.envoyer("L'avion est deja en vol : c'est trop tard pour ca.")
        if "suivi" in ordres:
            suivi_min, t_suivi = ordres.pop("suivi"), 0
        if ordres.pop("repetition", None):
            if app is not None or phase not in ("MONTEE", "CROISIERE"):
                TG.envoyer("Repetition possible seulement en montee ou en croisiere, pas pendant une approche.")
            else:
                app, raison = preparer_repetition(sim, a, avion)
                if app:
                    cap_repetition = (round(num(sim.lire("AUTOPILOT_HEADING_LOCK_DIR"))) % 360
                                      if lateral == "FIXE" else None)
                    gaz_a_rendre = num(sim.lire("GENERAL_ENG_THROTTLE_LEVER_POSITION:1"), None)
                    phase, t_verif = "APPROCHE", 0
                    mode_cap(sim, round(math.degrees(num(sim.lire("PLANE_HEADING_DEGREES_MAGNETIC")))) % 360)
                else:
                    TG.envoyer(f"Pas de repetition : {raison}.")
        if "atterrissage" in ordres or "piste" in ordres:
            voulu, piste = ordres.pop("atterrissage", None), ordres.pop("piste", None)
            if app is not None or phase == "REMISE":
                TG.envoyer("Trop tard : l'approche est deja commencee.")
            else:
                a.atterrissage = a.atterrissage if voulu is None else voulu
                a.piste = piste or a.piste
                if voulu and not a.sens:
                    TG.envoyer("Je n'ai pas pu mesurer le sens des gouvernes au sol pendant ce vol : je ferai "
                               "l'approche et je te rendrai l'avion a 3 NM de la piste.")
        nouvelle = ordres.pop("altitude", None)
        descente_demandee = ordres.pop("descente", None)
        if (nouvelle or descente_demandee) and phase not in ("MONTEE", "CROISIERE"):
            TG.envoyer("Trop tard pour ca : je suis deja en descente vers l'arrivee.")
            nouvelle = descente_demandee = None
        origine = ""
        if nouvelle and getattr(a, "suivre_controle", False):
            a.suivre_controle = False       # tu as choisi l'altitude : je ne suis plus le controle
            origine = " Je ne suis plus les altitudes du controle."
        # Toutes les 5 s : le controle a-t-il assigne une autre altitude ? (lue deux fois de suite)
        t_controle += 1
        if getattr(a, "suivre_controle", False) and phase in ("MONTEE", "CROISIERE") and t_controle >= 5:
            t_controle = 0
            assignee = altitude_controle(sim)
            if (assignee and assignee == controle_prec and abs(assignee - a.croisiere) >= 100
                    and (a.approche is None or assignee > a.approche + 500)):
                nouvelle, origine = assignee, " (donnee par le controle)"
            controle_prec = assignee
        if nouvelle:
            a.croisiere = nouvelle
            taux = a.vs_montee if nouvelle > alt else -max(a.vs_montee, 500)
            engager_vertical(sim, nouvelle, taux)
            phase, t_verif, echecs = "MONTEE", 0, 0
            dire(f"Nouvelle altitude de croisiere : {nouvelle} ft.{origine}", tel=True)

        # Rappel dix minutes avant de rendre la main.
        gs_kt = num(sim.lire("GPS_GROUND_SPEED")) * 1.944
        if (not prevenu_10_min and dist is not None and gs_kt > 40 and phase != "REMISE"
                and (dist - a.remise) / gs_kt * 60 <= 10):
            prevenu_10_min = True
            dire(f"Je te rends l'avion dans environ 10 minutes ({dist:.0f} NM de l'arrivee).", tel=True)

        # Le pilote a coupe l'AP : il reprend la main, on s'arrete.
        # Il faut trois lectures "coupe" de suite : une lecture manquee ne compte pas.
        ap_coupe = ap_coupe + 1 if booleen(sim.lire("AUTOPILOT_MASTER")) is False else 0
        if ap_coupe >= 3:
            dire("Pilote automatique coupe : tu as les commandes. Je m'arrete la.", alerte=True)
            return
        # Simulateur en pause : rien ne bouge, inutile de verifier, relancer ou noter quoi que ce soit.
        if fige:
            continue

        # Cap : toutes les 3 s, on revise le cap vers le prochain point du plan de vol.
        t_cap += 1
        if lateral == "CAP" and t_cap >= 3 and phase != "APPROCHE":      # en approche, c'est l'axe de piste
            t_cap = 0
            cap = cap_route(sim)
            if cap is not None:
                if cap_envoye is not None:
                    # L'avion a-t-il bien pris le cap envoye la fois d'avant ?
                    affiche = num(sim.lire("AUTOPILOT_HEADING_LOCK_DIR"), None)
                    ecart = None if affiche is None else abs((affiche - cap_envoye + 180) % 360 - 180)
                    ok = (booleen(sim.lire("AUTOPILOT_HEADING_LOCK")) is not False
                          and (ecart is None or ecart <= 3))
                    cap_refuse = 0 if ok else cap_refuse + 1
                    if cap_refuse == 5:
                        dire("L'avion ne prend pas mes ordres de cap : regle le cap ou le mode NAV toi-meme.", alerte=True)
                    if not ok:
                        mode_cap(sim, cap)
                if cap != cap_envoye:
                    sim.envoyer("HEADING_BUG_SET", cap)
                    if cap_envoye is None or abs((cap - cap_envoye + 180) % 360 - 180) >= 10:
                        dire(f"Nouveau cap {cap:03d}.")
                    cap_envoye = cap

        train.pas(sim)                       # train et volets : on insiste jusqu'a confirmation
        volets.pas(sim)
        # Altimetre recale chaque minute (pression locale, standard au-dessus de 18000 ft).
        t_baro += 1
        if t_baro >= 60 and not fige:
            t_baro = 0
            sim.envoyer("BAROMETRIC")
        # Phares d'atterrissage : eteints au-dessus de 10000 ft, rallumes en dessous.
        voulu = alt < 10000
        if phares is not None and voulu != phares:
            sim.envoyer("LANDING_LIGHTS_ON" if voulu else "LANDING_LIGHTS_OFF")
        phares = voulu
        # Suivi automatique sur Telegram, si demande avec 'suivi 10'.
        t_suivi += 1
        if suivi_min and t_suivi >= suivi_min * 60 and phase != "REMISE" and not fige:
            t_suivi = 0
            TG.envoyer(texte_etat())
        # Carburant : autonomie au debit du moment, comparee au temps de vol restant (en croisiere).
        t_carburant += 1
        if t_carburant >= 60:
            t_carburant = 0
            infos["autonomie"] = autonomie_h(sim)
            if (phase == "CROISIERE" and infos["autonomie"] and dist is not None and gs_kt > 40
                    and infos["autonomie"] < dist / gs_kt + 0.33):
                dire(f"Carburant juste : {infos['autonomie'] * 60:.0f} min d'autonomie pour "
                     f"{dist / gs_kt * 60:.0f} min de vol restantes. Prevois de te poser avant.", alerte=True)
        # Montee automatique : toutes les 10 s, le taux suit la vitesse (trop lent : on monte moins vite).
        t_taux += 1
        if a.taux_auto and phase == "MONTEE" and t_taux >= 10 and not fige and alt < a.croisiere - 500:
            t_taux = 0
            ancien = a.vs_montee
            if ias < v_plancher:
                a.vs_montee = max(a.vs_montee - 500, 200)
            elif ias < v_montee - 5:
                a.vs_montee = max(a.vs_montee - 200, 300)
            elif ias > v_montee + 8 and (ias_avant is None or ias >= ias_avant - 1):
                a.vs_montee = min(a.vs_montee + 200, 4000)
            ias_avant = ias
            if a.vs_montee != ancien:
                regler_vs(sim, a.vs_montee)

        # Securites
        if sim.lire("STALL_WARNING"):
            dire("ALERTE DECROCHAGE : reprends la main.", alerte=True)
        if abs(math.degrees(num(sim.lire("PLANE_BANK_DEGREES")))) > 40:
            dire("Inclinaison anormale : reprends la main.", alerte=True)

        if t_log >= 60:
            t_log = 0
            d = f"{dist:.0f} NM" if dist is not None else "distance inconnue"
            dire(f"[{phase}] {alt:.0f} ft, {ias:.0f} kt, {vs:+.0f} ft/min, {d}, {estimation(sim, dist, a.remise)}")

        # Atterrissage automatique : l'approche commence a 18 NM de l'aeroport.
        if phase == "DESCENTE" and a.atterrissage and app is None and dist is not None and dist <= a.remise + 8:
            app, raison = preparer_approche(sim, a, avion)
            if app:
                phase, t_verif = "APPROCHE", 0
                mode_cap(sim, round(math.degrees(num(sim.lire("PLANE_HEADING_DEGREES_MAGNETIC")))) % 360)
            else:
                a.atterrissage = False
                dire(f"Pas d'atterrissage automatique : {raison}. Je te rendrai l'avion a {a.remise:.0f} NM.",
                     alerte=True)

        if phase == "MONTEE":
            if abs(alt - a.croisiere) < 150:
                phase = "CROISIERE"
                if gaz_a_rendre is not None:                 # apres une repetition : gaz d'avant rendus
                    sim.envoyer("THROTTLE_SET", gaz_a_rendre / 100 * 16383)
                    gaz_a_rendre = None
                if tod is None:
                    dire(f"Croisiere a {a.croisiere} ft. Je n'ai pas encore trouve l'arrivee dans le plan de "
                         "vol : je la lirai sur la derniere branche. Pour ne pas attendre, envoie "
                         "'arrivee LFMD' (le code de ton aeroport).", tel=True)
                else:
                    dire(f"Croisiere a {a.croisiere} ft. Debut de descente prevu a {tod:.0f} NM de l'arrivee. "
                         f"Estimation : {estimation(sim, dist, a.remise)}.", tel=True)
            elif t_verif >= 30:
                t_verif = 0
                if vs < 150 and alt < a.croisiere - 300:   # l'avion ne monte pas : on renvoie les ordres
                    echecs += 1
                    engager_vertical(sim, a.croisiere, a.vs_montee)
                    if echecs >= 3:
                        dire("L'avion ne monte pas (puissance ? avion non compatible ?).", alerte=True)
                        echecs = 0

        if phase in ("MONTEE", "CROISIERE"):
            if dist is None and phase == "CROISIERE" and not sans_distance_signale:
                sans_distance_signale = True
                dire("Distance a l'arrivee inconnue : je tiens la croisiere, "
                     "la descente sera a lancer par toi (ou relance avec --dest LAT LON).", alerte=True)
            if tod is None:
                if descente_demandee:
                    TG.envoyer("Je ne connais pas encore l'arrivee : envoie d'abord 'arrivee LFMD'.")
            elif descente_demandee or (dist is not None and dist <= tod):
                gs_kt = num(sim.lire("GPS_GROUND_SPEED")) * 1.944
                vs_desc = -min(max(gs_kt * 5.3, 800), 2500) if gs_kt > 40 else -1500   # pente ~3 degres
                # Vol trop court : a peine monte, deja au point de descente. On se met en palier
                # au lieu de replonger vers le sol juste apres le decollage.
                if not descente_demandee and phase == "MONTEE" and alt < a.approche + 1500:
                    a.approche = max(a.approche, int(math.ceil(alt / 100) * 100))
                    dire(f"Vol trop court pour monter puis descendre : je me mets en palier a {a.approche} ft "
                         "et je te rendrai l'avion a cette altitude.", tel=True)
                if a.approche > alt:
                    vs_desc = 500                          # palier un peu au-dessus : on finit de monter
                engager_vertical(sim, a.approche, vs_desc)
                phase, t_verif, vs_cmd, t_vs, echecs = "DESCENTE", 0, vs_desc, 0, 0
                # Gaz : on tient la vitesse du moment, sauf si une automanette s'en occupe.
                if a.gaz and not sim.lire("AUTOPILOT_THROTTLE_ARM"):
                    vitesse = {
                        "cible": a.vitesse_descente or ias,
                        "gaz": num(sim.lire("GENERAL_ENG_THROTTLE_LEVER_POSITION:1"), 70) / 100,
                        "prec": ias,
                    }
                reste = f"{dist:.0f} NM restants" if dist is not None else "distance inconnue"
                dire(f"Descente vers {a.approche} ft a {vs_desc:.0f} ft/min ({reste}).", tel=True)

        elif phase == "DESCENTE":
            if vitesse:
                tenir_vitesse(sim, vitesse, ias)
            if sim.lire("OVERSPEED_WARNING"):
                dire("Survitesse : reduis les gaz ou sors les aerofreins.", alerte=True)
            # Relief : a moins de 1500 ft du sol alors qu'on descend encore, on arrete la descente.
            hauteur = sim.lire("PLANE_ALT_ABOVE_GROUND")
            if (hauteur is not None and num(hauteur) < 1500 and alt > a.approche + 300
                    and vs < -200 and not relief_signale):
                relief_signale = True
                a.approche = int(math.ceil((alt + 200) / 100) * 100)
                vs_cmd = 500
                engager_vertical(sim, a.approche, vs_cmd)
                dire(f"Relief a moins de 1500 ft sous l'avion : j'arrete la descente et je tiens {a.approche} ft. "
                     "Je te rendrai l'avion a cette altitude.", alerte=True)
            if t_verif >= 30 and alt > a.approche + 300:
                t_verif = 0
                if vs > -150:      # l'avion ne descend pas : on renvoie les ordres
                    engager_vertical(sim, a.approche, vs_cmd)
                    echecs += 1
                    if echecs >= 2:
                        dire("L'avion ne descend pas malgre mes ordres : lance la descente toi-meme "
                             "(molette VS du pilote automatique).", alerte=True)
                else:
                    echecs = 0
            # Toutes les 8 s, on recalcule le taux de descente qu'il faut pour etre a
            # l'altitude voulue 3 NM avant le point ou je te rends l'avion.
            t_vs += 1
            if t_vs >= 8 and dist is not None and gs_kt > 40 and alt > a.approche + 300:
                t_vs = 0
                minutes = max((dist - a.remise - 3) / gs_kt * 60, 0.5)
                besoin = -(alt - a.approche) / minutes
                if besoin < -a.vs_descente_max * 1.2 and not trop_haut_signale:
                    trop_haut_signale = True
                    dire("Je serai trop haut a l'arrivee meme en descendant au maximum : "
                         "prevois un tour d'attente ou une descente plus forte.", alerte=True)
                besoin = round(max(min(besoin, -500), -a.vs_descente_max), -2)
                if abs(besoin - vs_cmd) >= 100:
                    vs_cmd = besoin
                    regler_vs(sim, vs_cmd)
            # Avec un cap fixe l'avion peut passer a cote de l'arrivee : on le detecte
            # quand la distance se remet a augmenter.
            if dist is not None:
                dist_min = dist if dist_min is None else min(dist_min, dist)
            s_eloigne = dist is not None and dist > dist_min + 2
            if (dist is not None and dist <= a.remise) or s_eloigne \
                    or (dist is None and abs(alt - a.approche) < 150):
                en_pause = a.pause and (sim.envoyer("PAUSE_ON") or sim.envoyer("PAUSE_SET", 1))
                ou = "distance a l'arrivee inconnue" if dist is None else (
                    f"l'avion s'eloigne de l'arrivee ({dist:.0f} NM)" if s_eloigne
                    else f"a {dist:.0f} NM de l'arrivee")
                dire(f"Fin de mon travail, {ou} : {alt:.0f} ft, {ias:.0f} kt. "
                     + ("SIMULATEUR EN PAUSE. Enleve la pause quand tu es pret, puis coupe "
                        "le pilote automatique pour reprendre la main." if en_pause else
                        "A TOI : coupe le pilote automatique quand tu es pret."), alerte=True)
                if a.pause and not en_pause:
                    dire("Je n'ai pas reussi a mettre le simulateur en pause.", alerte=True)
                phase, t_verif = "REMISE", 0

        elif phase == "APPROCHE":
            if vitesse is None and a.gaz and not sim.lire("AUTOPILOT_THROTTLE_ARM"):
                vitesse = {"cible": ias, "prec": ias,
                           "gaz": num(sim.lire("GENERAL_ENG_THROTTLE_LEVER_POSITION:1"), 70) / 100}
            action = approche_pas(sim, a, app, vitesse)
            if vitesse:
                tenir_vitesse(sim, vitesse, ias)
            if action == "arrondi":
                action = arrondir_et_rouler(sim, a, app, vitesse)    # remise de gaz deja faite s'il le faut
                if action == "pose":
                    vs_t, ecart_t, arret = app["bilan"]
                    dire(f"Pose sur la piste {app['piste']['ident']} a {vs_t:.0f} ft/min, {ecart_t:+.0f} m de l'axe"
                         + (f", arrete a {arret:.0f} m du seuil" if arret is not None else "")
                         + ". Frein de parking serre. A toi pour degager la piste.", tel=True)
                    return
            elif action == "remise":
                remettre_gaz(sim, a, app, app["raison"])
            if action in ("repetition", "remise") and app["piste"].get("fictive"):
                if action == "repetition":
                    vs_t, ecart_t, ias_t, duree = app["bilan"]
                    avis = ("toucher dur : l'arrondi est trop mou" if vs_t < -400 else
                            "l'avion flotte : l'arrondi est trop fort" if duree > 14 else "bon arrondi")
                    dire(f"Repetition terminee : toucher a {vs_t:.0f} ft/min, {ecart_t:+.0f} m de l'axe, "
                         f"{ias_t:.0f} kt, arrondi de {duree:.0f} s ({avis}). Mesures dans le dernier fichier "
                         f"atterrissage_*.csv. Je remonte a {a.croisiere} ft et je reprends le vol.", tel=True)
                # Reprise du vol : train et volets rentres, route et croisiere reprises, gaz rendus en croisiere.
                train, volets, vitesse, app = Train(), Volets(), None, None
                if lateral == "FIXE" and cap_repetition is not None:
                    mode_cap(sim, cap_repetition)
                else:
                    lateral = engager_lateral(sim, a)
                phase, t_verif, echecs = "MONTEE", 0, 0
            elif action == "remise":
                phase, vitesse = "REMISE", None
            elif action == "main":
                en_pause = a.pause and (sim.envoyer("PAUSE_ON") or sim.envoyer("PAUSE_SET", 1))
                dire(f"Approche de la piste {app['piste']['ident']} : 3 NM du seuil, {alt:.0f} ft, {ias:.0f} kt, "
                     "train et volets sortis. " + ("SIMULATEUR EN PAUSE. Enleve la pause, coupe le pilote "
                                                   "automatique et pose-toi." if en_pause else
                                                   "A TOI : coupe le pilote automatique et pose-toi."), alerte=True)
                phase, t_verif = "REMISE", 0

        elif phase == "REMISE":
            if vitesse and not fige:
                tenir_vitesse(sim, vitesse, ias)
            if t_verif >= 30 and not fige:
                t_verif = 0
                dire("J'attends que tu reprennes la main (coupe le pilote automatique).", alerte=True, tel=False)
            if getattr(sim, "fictif", False) and sim.dist < a.remise - 3:
                return   # fin de l'essai fictif


def connecter(ordres):
    """Se connecte au simulateur, en reessayant tant qu'il n'est pas lance."""
    prevenu = False
    while True:
        try:
            return SimReel()
        except Exception as e:
            if not prevenu:
                prevenu = True
                dire(f"Simulateur introuvable ({e}). Lance MSFS et charge ton vol ; je reessaie "
                     "toutes les 10 secondes. Envoie 'stop' pour annuler.", tel=True)
        for _ in range(10):
            traiter_commandes(None, lambda: "J'attends que MSFS soit lance.", ordres)
            if ordres.pop("stop", None):
                return None
            time.sleep(1)


def main():
    p = argparse.ArgumentParser(description="Bot pilote pour MSFS, commande depuis Telegram")
    p.add_argument("--croisiere", type=int, help="vol direct sans Telegram : altitude de croisiere en pieds")
    p.add_argument("--approche", type=int, help="vol direct sans Telegram : altitude de remise en pieds")
    p.add_argument("--cap", type=int, help="vol direct sans Telegram : cap a garder (sinon plan de vol)")
    p.add_argument("--remise", type=float, default=10, help="distance de l'arrivee ou je te rends la main (NM)")
    p.add_argument("--vs-montee", type=int, default=700, help="taux de montee en ft/min (700 petit avion, 1800 jet)")
    p.add_argument("--vr", type=int, default=90,
                   help="vitesse a laquelle cabrer au decollage automatique (kt ; 90 pour le Vision Jet)")
    p.add_argument("--vs-descente-max", type=int, default=3000, help="taux de descente maximal en ft/min")
    p.add_argument("--dest", type=float, nargs=2, metavar=("LAT", "LON"),
                   help="coordonnees de l'arrivee en degres, si le plan de vol GPS n'est pas lu")
    p.add_argument("--vitesse-descente", type=float, help="vitesse a tenir en descente (kt) ; defaut : celle de croisiere")
    p.add_argument("--sans-pause", dest="pause", action="store_false",
                   help="ne pas mettre le simulateur en pause au moment de te rendre la main")
    p.add_argument("--sans-telegram", dest="telegram", action="store_false", help="ne pas utiliser Telegram")
    p.add_argument("--nav", action="store_true",
                   help="laisser le mode NAV de l'avion suivre la route au lieu de piloter au cap")
    p.add_argument("--sans-gaz", dest="gaz", action="store_false", help="ne jamais toucher a la manette des gaz")
    p.add_argument("--test", action="store_true", help="essai avec un avion et un Telegram fictifs")
    a = p.parse_args()
    executer(a)


def executer(a, tg_test=None):
    global TG
    direct = a.croisiere is not None and a.approche is not None
    ordres = {}
    a.taux_auto, a.suivre_controle, a.terrain = False, False, None   # vol direct : tout est donne a la main
    a.code_arrivee = a.alt_terrain = a.piste = None
    a.atterrissage, a.sens, a.hauteur_sol = False, {}, 3.0

    if a.test:
        TG = tg_test or TelegramFictif()
        # L'essai ne doit toucher ni aux vrais reglages ni au vrai journal.
        global REGLAGES, JOURNAL
        import tempfile
        REGLAGES = os.path.join(tempfile.gettempdir(), "reglages_essai_pilote.json")
        JOURNAL = os.path.join(tempfile.gettempdir(), "journal_essai_pilote.txt")
    else:
        # Un seul bot a la fois : deux exemplaires se disputeraient l'avion et Telegram.
        verrou = socket.socket()
        try:
            verrou.bind(("127.0.0.1", 47811))
        except OSError:
            sys.exit("Le bot tourne deja sur ce PC (peut-etre en arriere-plan).")
        executer.verrou = verrou
        if a.telegram:
            tg = Telegram(os.path.join(os.path.dirname(os.path.abspath(__file__)), "telegram.json"))
            if sys.stdin is None or not sys.stdin.isatty():
                if not tg.jeton:      # lance sans fenetre : impossible de demander le jeton
                    sys.exit("Jeton Telegram absent : lance d'abord lancer_bot.bat une fois.")
            if tg.configurer():
                TG = tg

    try:
        if direct:
            sim = SimFictif() if a.test else connecter(ordres)
            if sim:
                voler(sim, a, ordres)
            return
        if isinstance(TG, TelegramMuet) and not isinstance(TG, (Telegram, TelegramFictif)):
            sys.exit("Sans Telegram, donne les altitudes : python pilote_auto.py --croisiere 9000 --approche 3300")

        dire("Bot pilote pret. Envoie par exemple : vol 9000 3300", tel=True)
        vs_defaut, dest_defaut = a.vs_montee, a.dest
        while True:
            traiter_commandes(None, lambda: "En attente. Envoie par exemple : vol 9000 3300", ordres)
            au_sol = [f for nom, f in (("prepare", preparer), ("moteur", demarrer_moteur)) if ordres.pop(nom, None)]
            if au_sol:
                sim = SimFictif() if a.test else connecter(ordres)
                if sim:
                    for f in au_sol:
                        f(sim)
                    if not a.test:
                        try:
                            sim.sm.exit()
                        except Exception:
                            pass
            demande = ordres.pop("vol", None)
            if not demande:
                time.sleep(0 if a.test else 1)
                continue
            a.croisiere, a.approche, a.cap = demande["croisiere"], demande["approche"], demande["cap"]
            a.taux_auto = demande["taux"] == "auto"
            a.vs_montee = 700 if a.taux_auto else demande["taux"] or vs_defaut
            a.dest, a.terrain = demande["dest"] or dest_defaut, demande["terrain"]
            a.suivre_controle = demande["controle"]
            a.code_arrivee, a.alt_terrain, a.piste = demande["code"], demande["alt_terrain"], None
            a.atterrissage, a.sens = lire_reglages().get("atterrissage") == "oui", {}
            sim = SimFictif(decollage_auto=getattr(a, "test_decollage", False)) if a.test else connecter(ordres)
            if sim is None:
                dire("Vol annule.", tel=True)
                continue
            voler(sim, a, ordres)
            if a.test:
                return
            try:
                sim.sm.exit()
            except Exception:
                pass
            dire("Vol termine. Envoie 'vol 9000 3300' pour le suivant.", tel=True)
    except KeyboardInterrupt:
        dire("Bot arrete. Le pilote automatique de l'avion reste dans son etat actuel.")


if __name__ == "__main__":
    main()
