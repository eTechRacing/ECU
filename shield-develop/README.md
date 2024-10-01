# SHIELD

ADCAN files moved to ADCAN-Legacy branch

_Pendent de publicar en angles

La shield ha de ser una placa versàtil i robusta que serveixi per implementar fàcilment elements comuns que comparteixen diverses plaques. L’avantatge d’implementar una shield és la de disposar d’una base coneguda, provada i fiable per, 
a partir d’ella, implementar qualsevol altre sistema que s’hagi d’incloure dins del cotxe.
Les funcionalitats de la placa shield passen, principalment, per una alimentació estable i constant i per un bus de comunicacions amplament validat i funcional. 
Tot això rodejant d’un microcontrolador molt consistent encarregat de l’adquisició, el processament i l’enviament de dades.

Requisits:
-Alimentació a 5 V i 3.3 V a partir de LV
-Comunicació via CAN1 i CAN2
-Connectors plug-and-play
-MCU potent (STM23)
-Documentació
-Reduir EMIs i soroll amb un bon disseny

El conntector entre la shield i la carrier és una de les parts fonamentals de la placa. La idea és que hi hagi connexió electrica i mecànica entre elles.
S'opta per un connector board-to-board tipus pin header (fabricant Würth) ja que dona la opció d'encaixar una placa sobre l'altre i, a mes, de disposar de totes les connexions
amb un connector senzill de posar i treure.

El microcontrolador és el més potent que s'ha pogut aconseguir per patrocinadors. Més concretament el STM32F777VI6T que compleix de sobres tots els requisits. De totes maneres,
pensant en el fet de que la shield sigui una placa amb futur, s'afageix una memoria EEPROM utilitzat el I2C número 4 del STM32.

Pel que fa la resta de la placa no hi ha cap complicació. S'implementa un USB_OTG amb un connector micro USB 2.0 per debug i visualització de dades per PC. Es fa servir el CAN
transceiver MCP2551 de microchip (fàcil d'aconseguir per samples) i unes proteccions i filtres a la linea CAN bastant estàndard. La resta dels pins del MCU van directament al
pin header excepte:
-VBAT: es col·loca un divisor de tensió rebaixant 30V a 3.3V. Això permet fer servir VBAT com una entrada analogica per medir la tensió que ofereix la LV.
-VBUS: es fa servir com a entrada digital per detectar si hi ha presencia d'USB.
-NRST: pin de reset del micocontrolador. Necessari per programar. Se li posa un botó per també poder fer reset manualment.
-Boot0: se li posa un switch per curtcircuitar a 3.3V o a GND. Això permetrà setejar al micro com a blank i poder carregar informació des del USB o programar més facilment.
