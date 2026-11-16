"""
Programme Python :
    Recherche des postions des potentiomètres digitaux 
    pour répondre à la consigne de gain voulue.
    
Last Update : 06/01/2026
Auteur : Yoann LAI KOUN SING

Conclusion:
    Le gain maximum est de 127 pour des potentiomètre digitaux de 50 000 ohm.

"""


def Research_Gain(Gain_voulu):
    Resultat_Gain = [0.0, 0, 0, 0]
    Resistance1 = 0.0
    Resistance2 = 0.0
    Resistance3 = 0.0
    Gain_calcule = 0.0
    Resultat = 0.0
    rAB_P1 = 48600
    rAB_P2 = 46200
    rAB_P3 = 47000
    rWiper_P1 = 124
    rWiper_P2 = 309
    rWiper_P3 = 125
    maxPositions = 255
    Res1 = 10000
    Res4 = 1000

    for i in range(0,256):
        Resistance1 = ((rAB_P1 * i) / maxPositions ) + rWiper_P1
        for j in range(0,256):
            Resistance2 = ((rAB_P2 * j) / maxPositions ) + rWiper_P2
            for k in range(0,256):
                Resistance3 = ((rAB_P3 * k) / maxPositions ) + rWiper_P3
                Gain_calcule = (1+ Resistance3/Resistance2)*( Resistance1 / (Res1+Resistance1) )        #Gain total
                #Gain_calcule = (1+ Resistance3/Resistance2)                    #Gain de l'AOP uniquement

                if ( abs(Gain_voulu - Gain_calcule) < abs(Gain_voulu - Resultat_Gain[0]) ):
                    Resultat_Gain[0] = Gain_calcule
                    Resultat_Gain[1] = i
                    Resultat_Gain[2] = j
                    Resultat_Gain[3] = k

    return Resultat_Gain



Tab_Res = {0,0,0,0}
Tab_Res = Research_Gain(1)
print("Gain: " + str(Tab_Res[0]))
print("pos1: " + str(Tab_Res[1]))
print("pos2: " + str(Tab_Res[2]))
print("pos3: " + str(Tab_Res[3]))
Tab_Res = Research_Gain(30)
print("Gain: " + str(Tab_Res[0]))
print("pos1: " + str(Tab_Res[1]))
print("pos2: " + str(Tab_Res[2]))
print("pos3: " + str(Tab_Res[3]))
Tab_Res = Research_Gain(60)
print("Gain: " + str(Tab_Res[0]))
print("pos1: " + str(Tab_Res[1]))
print("pos2: " + str(Tab_Res[2]))
print("pos3: " + str(Tab_Res[3]))
Tab_Res = Research_Gain(90)
print("Gain: " + str(Tab_Res[0]))
print("pos1: " + str(Tab_Res[1]))
print("pos2: " + str(Tab_Res[2]))
print("pos3: " + str(Tab_Res[3]))
Tab_Res = Research_Gain(120)
print("Gain: " + str(Tab_Res[0]))
print("pos1: " + str(Tab_Res[1]))
print("pos2: " + str(Tab_Res[2]))
print("pos3: " + str(Tab_Res[3]))
Tab_Res = Research_Gain(150)
print("Gain: " + str(Tab_Res[0]))
print("pos1: " + str(Tab_Res[1]))
print("pos2: " + str(Tab_Res[2]))
print("pos3: " + str(Tab_Res[3]))
