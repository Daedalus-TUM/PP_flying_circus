////////////////////////////////////////////////////

Arduino-Script zum manuellen Senden von Steuerbefehlen
Zur Verwendung auf passenden Arduino installieren
und in der Konsole die gewünschten Werte eingeben.

////////////////////////////////////////////////////

//REFERENCE: Funkprotokoll |  30  |  0  |Distanz|Distanz|Nordwinkel|Nordwinkel|Sollhoehe|Abwurf|Gondelwinkel|Gondelwinkel|
//                         |   0  |  1  |   2   |   3   |     4    |      5   |     6   |   7  |      8     |     9      |
//
//Nordwinkel:    Winkel zwischen norwärts gerichtetem Vektor und Vektor zum nächsten Wegpunkt
//Distanz:       absolute Distanz zwischen Gondel und nächstem Wegpunkt
//Gondelwinkel:  Winkel zwischen norwärts gerichtetem Vektor und Richtungswinkel der Gondel

////////////////////////////////////////////////////

Waehrend des Betriebs sind die folgenden Werte mit 'Enter' einzugeben:
W, S:     Einstellen der Sollhoehe
R, F:     Eingabe von Distanz und Nordwinkel.
Y:        Reset
X:        wirft Paket ab

