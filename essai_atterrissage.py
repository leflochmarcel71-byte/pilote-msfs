"""Essai de l'atterrissage automatique sans le simulateur : un avion fictif qui vire, monte,
descend, se pose et freine, de 60 NM au nord-ouest de Cannes jusqu'a l'arret sur la piste 17.
Lancement : python essai_atterrissage.py"""
import argparse
import math
import tempfile

import pilote_auto as p

DECL = 2.0                     # declinaison magnetique fictive (cap vrai - cap magnetique)
R = 6371000


class SimPiste(p.SimFictif):
    def __init__(self, depart_nm=60, vent=(200, 12), ailerons_muets=False, titre="Essai", ligne=False,
                 code="LFMD", piste="17"):
        super().__init__(distance_nm=depart_nm)
        self.ailerons_muets = ailerons_muets          # avion dont on ne peut pas lire les ailerons
        self.titre = titre                            # nom de l'avion dans le simulateur
        # Avion de ligne : biréacteur avec automanette, ILS a l'arrivee et autoland.
        self.ligne, self.at, self.spd, self.spd_tenue, self.apr, self.glide = ligne, False, 0, False, False, False
        self.envois = []                              # (commande, valeur, vitesse, au sol) envoyees par le bot
        ap = p.chercher_aeroport(code)
        self.lat0, self.lon0 = ap["lat"], ap["lon"]
        self.piste = p.choisir_piste(code, voulue=piste)
        self.sol = self.piste["alt_ft"]
        self.alt = self.sol
        # Depart au nord-ouest, nez vers l'aeroport.
        self.x, self.y = -depart_nm * p.NM * 0.6, depart_nm * p.NM * 0.8
        self.cap = math.degrees(math.atan2(-self.x, -self.y)) % 360
        self.vent = vent
        self.conception["DESIGN SPEED VS0"] = 52.0
        if ligne:
            self.conception.update({"DESIGN SPEED VS0": 105.0, "DESIGN SPEED VS1": 120.0,
                                    "DESIGN TAKEOFF SPEED": 145.0, "DESIGN SPEED CLIMB": 280.0})
        self.v_cabre, self.v_max_decollage = (140, 165) if ligne else (70, 110)
        self.aileron, self.incl, self.volets_max = 0.0, 0.0, 3
        self.manuel, self.toucher_vs, self.crash = False, None, False

    def lat_lon(self):
        return (self.lat0 + math.degrees(self.y / R),
                self.lon0 + math.degrees(self.x / (R * math.cos(math.radians(self.lat0)))))

    @property
    def dist(self):
        return p.distance_nm(*self.lat_lon(), self.lat0, self.lon0) if hasattr(self, "lat0") else 0.0

    @dist.setter
    def dist(self, _):
        pass                                            # calculee d'apres la position

    def lire(self, nom):
        lat, lon = self.lat_lon()
        au_sol = self.alt <= self.sol + 0.5
        propre = {
            "SIM_ON_GROUND": au_sol, "PLANE_ALT_ABOVE_GROUND": self.alt - self.sol,
            "PLANE_ALTITUDE": self.alt, "INDICATED_ALTITUDE": self.alt,
            "PLANE_LATITUDE": math.radians(lat), "PLANE_LONGITUDE": math.radians(lon),
            "PLANE_HEADING_DEGREES_TRUE": math.radians(self.cap),
            "PLANE_HEADING_DEGREES_MAGNETIC": math.radians(self.cap - DECL),
            "PLANE_BANK_DEGREES": math.radians(self.incl),            # ici, positif a droite
            "GPS_WP_BEARING": math.radians((p.cap_vers(lat, lon, self.lat0, self.lon0) - DECL) % 360),
            "GPS_GROUND_SPEED": self.vitesse_sol() * 0.5144, "GROUND_VELOCITY": self.vitesse_sol(),
            "AMBIENT_WIND_DIRECTION": self.vent[0], "AMBIENT_WIND_VELOCITY": self.vent[1],
            "NAV_HAS_LOCALIZER:1": self.ligne, "NAV_HAS_GLIDE_SLOPE:1": self.ligne,
            "NAV_LOCALIZER:1": (self.piste["cap"] - DECL) % 360,
            "AUTOPILOT_APPROACH_HOLD": self.apr, "AUTOPILOT_GLIDESLOPE_HOLD": self.glide,
            "AUTOPILOT_THROTTLE_ARM": self.at, "AUTOPILOT_AIRSPEED_HOLD": self.spd_tenue,
            "AIRSPEED_MACH": self.ias * (1 + self.alt / 1000 * 0.02) / 661,
            "NUMBER_OF_ENGINES": 2 if self.ligne else 1, "GENERAL_ENG_COMBUSTION:2": self.moteur,
            "OVERSPEED_WARNING": self.ias > (340 if self.ligne else 180),
            "AILERON_POSITION": None if self.ailerons_muets else self.aileron,
            "FLAPS_NUM_HANDLE_POSITIONS": self.volets_max,
            "GEAR_HANDLE_POSITION": self.train, "TITLE": self.titre.encode(),
        }
        return propre[nom] if nom in propre else super().lire(nom)

    lire_vite = lire

    def vitesse_sol(self):
        """Vitesse sol en kt : vitesse air corrigee du vent (sauf au sol)."""
        if self.alt <= self.sol + 0.5:
            return self.ias
        vent = math.radians(self.vent[0])
        vx = self.ias * math.sin(math.radians(self.cap)) - self.vent[1] * math.sin(vent)
        vy = self.ias * math.cos(math.radians(self.cap)) - self.vent[1] * math.cos(vent)
        return math.hypot(vx, vy)

    def envoyer(self, nom, valeur=None):
        self.envois.append((nom, valeur, self.ias, self.alt <= self.sol + 0.5))
        if nom == "AUTO_THROTTLE_ARM" and self.ligne:
            self.at = not self.at
        elif nom == "AP_SPD_VAR_SET":
            self.spd = valeur
        elif nom == "AP_AIRSPEED_ON":
            self.spd_tenue = True
        elif nom == "AP_APR_HOLD_ON" and self.ligne:
            self.apr = True
        elif nom == "AILERON_SET":
            self.aileron = -valeur / 16383                 # sens inverse, comme les autres gouvernes
        elif nom == "AILERONS_RIGHT":
            self.aileron = min(self.aileron + 0.05, 1)
        elif nom in ("AUTOPILOT_OFF", "AP_MASTER"):
            self.ap, self.manuel = False, True
        elif nom == "AUTOPILOT_ON":
            self.ap, self.manuel = True, False
        elif nom == "FLAPS_INCR":
            self.volets = min(self.volets + 1, self.volets_max)
        elif nom == "GEAR_DOWN":
            self.train = True
        else:
            return super().envoyer(nom, valeur)
        return True

    def _pas(self, dt):
        if self.t > 4 * 3600:
            raise RuntimeError("essai trop long : l'avion ne s'est jamais pose")
        self.t += dt
        if self.pause:
            self.pause = max(self.pause - dt, 0)
            return
        au_sol = self.alt <= self.sol + 0.5
        if au_sol and (self.manuel or (self.ap and getattr(self, "en_vol", False))):   # roulage apres le toucher
            self.vs, self.incl = 0.0, 0.0
            freinage = 6 if self.freins else 0
            self.freins = max(self.freins - 1, 0)
            self.ias = max(self.ias - (1.5 + freinage) * dt + 3 * self.gaz * dt, 0)
            if self.ap:                                     # l'autoland tient l'axe au roulage
                self.cap = (self.cap + max(-3.0, min(3.0, p.angle(self.piste["cap"] - self.cap))) * dt) % 360
            else:
                self.cap = (self.cap + (-0.5 + 25 * self.palonnier * min(self.ias, 60) / 60) * dt) % 360
        elif self.ap:
            avant, ecart_m = p.sur_piste(self.piste, *self.lat_lon())
            if self.apr:                                    # mode APR : l'avion suit l'ILS
                err = p.angle(self.piste["cap"] - max(-25.0, min(25.0, ecart_m / 20)) - self.cap)
            else:
                err = p.angle(self.bug + DECL - self.cap) if self.hdg else 0.0
            virage = max(-3.0, min(3.0, err * 0.5))
            self.cap = (self.cap + virage * dt) % 360
            self.incl = virage * 8
            plan = self.piste["alt_ft"] + (avant + self.piste["visee_m"]) * math.tan(math.radians(3)) / 0.3048
            if self.apr and not self.glide and avant < 12 * p.NM and self.alt <= plan + 30:
                self.glide = True
            ecart = self.alt_sel - self.alt
            if self.glide:
                self.vs = max(-1500, min(0, -self.vitesse_sol() * 5.3 - 3 * (self.alt - plan)))
                hauteur = self.alt - self.sol
                if hauteur < 40:
                    self.vs = max(self.vs, -(150 + 10 * hauteur))     # arrondi de l'autoland
            elif self.mode_vs and abs(ecart) > 50 and ecart * self.vs_sel > 0:
                self.vs = self.vs_sel
            else:
                self.vs = max(-500, min(500, ecart * 5))
            if self.at and self.spd_tenue:
                cible = self.spd
            else:
                cible = 60 + 120 * self.gaz - self.vs / 40 - 8 * self.volets - (10 if self.train else 0)
            self.ias += (cible - self.ias) * min(0.05 * dt, 1)
        elif self.manuel and not au_sol:
            self.incl += 40 * self.aileron * dt
            self.cap = (self.cap + 1091 * math.tan(math.radians(self.incl)) / max(self.ias, 30) * dt
                        + 5 * self.palonnier * dt) % 360
            self.vs += ((-700 + 700 * self.gaz) + 3000 * self.profondeur - self.vs) * min(dt, 1)
            self.ias = max(self.ias + (3 * self.gaz - 1 - 2 * self.profondeur) * dt, 30)
        else:                                               # decollage fait par le pilote
            if not self.frein_park and self.t > 20:
                self.ias = min(self.ias + 3 * dt, self.v_max_decollage)
            self.vs = 900.0 if self.ias > self.v_cabre else 0.0
        v = self.ias * 0.5144
        vent = 0 if au_sol else self.vent[1] * 0.5144
        self.x += (v * math.sin(math.radians(self.cap)) - vent * math.sin(math.radians(self.vent[0]))) * dt
        self.y += (v * math.cos(math.radians(self.cap)) - vent * math.cos(math.radians(self.vent[0]))) * dt
        alt = self.alt + self.vs / 60 * dt
        if alt > self.sol + 5:
            self.en_vol = True
        if alt <= self.sol + 0.5 and getattr(self, "en_vol", False) and self.toucher_vs is None:
            self.toucher_vs = self.vs
            self.crash = self.vs < -900
        self.alt = max(alt, self.sol)


def reglages(sim, code="LFMD", piste="17"):
    terrain = p.chercher_aeroport(code)["alt_ft"]
    a = argparse.Namespace(croisiere=5000, approche=None, cap=None, remise=10, vs_montee=700, vr=90,
                           vs_descente_max=3000, dest=(sim.lat0, sim.lon0), vitesse_descente=None, pause=True,
                           nav=False, gaz=True, test=True, taux_auto=False, suivre_controle=False, terrain=None,
                           code_arrivee=code, alt_terrain=terrain, atterrissage=True, piste=piste)
    a.approche = int(round((terrain + p.HAUTEUR_REMISE_M / 0.3048) / 100) * 100)
    return a


def verifier_pose(sim):
    """Pose en douceur, arrete sur la piste, dans sa largeur, frein de parking serre."""
    avant, ecart = p.sur_piste(sim.piste, *sim.lat_lon())
    assert sim.toucher_vs is not None, "l'avion ne s'est pas pose (remise de gaz ?)"
    print(f"Toucher {sim.toucher_vs:.0f} ft/min, arret a {-avant:.0f} m du seuil, {ecart:+.0f} m de l'axe, "
          f"{sim.ias:.0f} kt")
    assert not sim.crash and sim.toucher_vs > -600, "toucher trop dur"
    assert sim.alt <= sim.sol + 0.5 and sim.ias < 3, "pas arrete au sol"
    assert 0 < -avant < sim.piste["longueur_m"], "arrete hors de la piste (avant le seuil ou apres la fin)"
    assert abs(ecart) < sim.piste["largeur_m"] / 2, "sorti sur le cote de la piste"
    assert sim.frein_park, "frein de parking pas serre"


def essai_ligne():
    """Biréacteur vers Nice : automanette, 250 kt sous 10000 ft, volets rentres cran par cran,
    aerofreins en descente, puis autoland sur l'ILS et arret sur la piste."""
    sim = SimPiste(depart_nm=120, vent=(0, 0), ligne=True, titre="Avion de ligne", code="LFMN", piste="04L")
    a = reglages(sim, code="LFMN", piste="04L")
    a.croisiere, a.vs_montee = 14000, 2000
    p.voler(sim, a, {})
    en_vol = [e for e in sim.envois if not e[3]]
    decr = [e[2] for e in en_vol if e[0] == "FLAPS_DECR"]
    print(f"Volets rentres a {', '.join(f'{v:.0f}' for v in decr)} kt ; vitesses visees "
          f"{sorted({e[1] for e in en_vol if e[0] == 'AP_SPD_VAR_SET'})}")
    assert decr and min(decr) >= 145 + 30 - 1, "volets rentres sous la vitesse prevue"
    assert not any(e[0] == "FLAPS_UP" for e in en_vol), "volets rentres d'un coup en vol"
    vitesses = {e[1] for e in en_vol if e[0] == "AP_SPD_VAR_SET"}
    assert {250, 280} <= vitesses, "250 kt sous 10000 ft ou 280 kt au-dessus jamais demandes"
    noms = [e[0] for e in en_vol]
    for attendu in ("AUTO_THROTTLE_ARM", "SPOILERS_ON", "SPOILERS_OFF", "SPOILERS_ARM_ON", "AP_APR_HOLD_ON"):
        assert attendu in noms, f"{attendu} jamais envoye en vol"
    verifier_pose(sim)


class TelegramEssai(p.TelegramMuet):
    """Envoie 'repetition' une fois l'avion en croisiere, et garde les messages du bot."""

    def __init__(self, sim):
        self.sim, self.envoye, self.messages = sim, False, []

    def envoyer(self, texte):
        self.messages.append(texte)

    def commandes(self):
        if not self.envoye and self.sim.alt >= 5950 and self.sim.ap:
            self.envoye = True
            return ["repetition"]
        return []


def essai_repetition():
    """En croisiere a 6000 ft : repetition sur la piste imaginaire, reprise du vol, puis vrai atterrissage."""
    sim = SimPiste(depart_nm=90)
    p.TG = tg = TelegramEssai(sim)
    a = reglages(sim)
    a.croisiere = 6000
    try:
        p.voler(sim, a, {})
    finally:
        p.TG = p.TelegramMuet()
    bilan = [m for m in tg.messages if m.startswith("Repetition terminee")]
    print(bilan[0] if bilan else "pas de bilan de repetition : " + " | ".join(tg.messages[-5:]))
    assert bilan, "la repetition n'a pas eu lieu"
    assert sim.toucher_vs is not None and sim.ias < 3, "pas pose a l'arrivee apres la repetition"


def essai_memoire():
    """Meme avion qu'avant, ailerons devenus illisibles : le sens retenu sert, il se pose quand meme."""
    sim = SimPiste(ailerons_muets=True)
    p.voler(sim, reglages(sim), {})
    assert sim.toucher_vs is not None and sim.ias < 3, "gouvernes retenues non reprises"
    print("Gouvernes retenues pour cet avion : pose quand meme")


def essai_sans_arrondi():
    """Gouvernes illisibles : approche faite, avion rendu a 3 NM, en l'air, sans se poser."""
    sim = SimPiste(ailerons_muets=True, titre="Autre avion")
    p.voler(sim, reglages(sim), {})
    avant, ecart = p.sur_piste(sim.piste, *sim.lat_lon())
    print(f"Rendu a {avant / p.NM:.1f} NM, {sim.alt - sim.sol:.0f} ft sol, {ecart:+.0f} m de l'axe")
    assert sim.toucher_vs is None and 2 * p.NM < avant < 4 * p.NM and abs(ecart) < 100, "approche non rendue a 3 NM"


def essai(vent=(200, 12)):
    sim = SimPiste(vent=vent)
    p.voler(sim, reglages(sim), {})
    verifier_pose(sim)


if __name__ == "__main__":
    # Journal et mesures de l'essai dans le dossier temporaire : le vrai journal reste propre.
    p.JOURNAL = p.os.path.join(tempfile.gettempdir(), "journal_essai_atterrissage.txt")
    p.REGLAGES = p.os.path.join(tempfile.gettempdir(), "reglages_essai_atterrissage.json")
    if p.os.path.exists(p.REGLAGES):
        p.os.remove(p.REGLAGES)                          # on part d'un avion inconnu
    essai()
    essai(vent=(0, 0))
    essai_memoire()
    essai_sans_arrondi()
    essai_ligne()
    essai_repetition()
    print("Essais d'atterrissage reussis.")
