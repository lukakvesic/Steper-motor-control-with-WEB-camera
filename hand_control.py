import cv2  # Uvoz biblioteke OpenCV za rad s kamerom
import mediapipe as mp  # Uvoz biblioteke Mediapipe za praćenje ruku
import time  # Uvoz biblioteke za rad s vremenom
import math  # Uvoz matematičke biblioteke za izračune
import numpy as np  # Uvoz biblioteke za rad s nizovima
import serial #Uvoz biblioteke za komunikaciju sa arduinom
import serial.tools.list_ports #Uvoz biblioteke za detalje komunikacijskih portova


# Podešavanje kamere za video snimanje
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)  # Postavite na 0 za korištenje  defoultne kamere 
cap.set(cv2.CAP_PROP_FPS, 120)



# Kreiranje klase "handDetector" za detekciju i praćenje ruku
class handDetector:
    def __init__(self, mode=False, MaxHands=1, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.MaxHands = MaxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.MaxHands, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, frame, draw=True):
        RGBframe = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Konvertovanje BGR slike u RGB sliku za Mediapipe
        self.results = self.hands.process(RGBframe)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, handLms, self.mpHands.HAND_CONNECTIONS)

        return frame

    def findPosition(self, frame, handNo=0, draw=True):
        lmsList = []

        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]

            for id, lm in enumerate(myHand.landmark):
                h, w, c = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmsList.append([id, cx, cy])
                if draw:
                    cv2.circle(frame, (cx, cy), 7, (240, 1, 0), cv2.FILLED)
        return lmsList
    
    def findFingerStatus(self, handLandmarks):
        # Određivanje statusa šake (otvorena ili zatvorena)
        # Proučite i prilagodite uvjete ovisno o vašem sustavu
        thumb_is_open = handLandmarks[4][2] < handLandmarks[3][2]
        index_is_open = handLandmarks[8][2] < handLandmarks[6][2]
        middle_is_open = handLandmarks[12][2] < handLandmarks[10][2]
        ring_is_open = handLandmarks[16][2] < handLandmarks[14][2]
        pinky_is_open = handLandmarks[20][2] < handLandmarks[18][2]

        #Broj prstiju koji su otvreni
        open_fingers = sum([thumb_is_open, index_is_open, middle_is_open, ring_is_open, pinky_is_open])

        return open_fingers






#Varijable vremena za racunanje fps
start_time = time.time()
pTime = 0

detector = handDetector(detectionCon=1)  # Kreiranje instance klase "handDetector"
prev_x = 0
direction_speed=0
prev_direction_speed = 0
hand_open = 0
prev_hand_open = 0

print("-----------------------")
time.sleep(1)

#Funkcija za ispis dostupnih portova
def list_ports():
    com_ports = list(serial.tools.list_ports.comports())
    print("-----------------------")
    if not com_ports:
        print("Nema dostupnih COM portova.")
    else:
        print("Dostupni COM portovi:")
        for port in com_ports:
            print(f"Port: {port.device}")
            print(f"Opis: {port.description}")
            print(f"ID proizvođača: {port.vid}")
            print(f"ID uređaja: {port.pid}")
            print("-----------------------")


list_ports()

print(" ")
com_port = input("Unesite vaš com port: ")

#Otvaranje serijske veze sa arduinom
try:
    ser = serial.Serial(com_port, 115200)
    print("Serijska veza uspostavljena.")
    time.sleep(1)  
except serial.SerialException as e:
    print(f"Greška pri uspostavi serijske veze: {e}")
except Exception as e:
    print(f"Nepredviđena greška: {e}")

while True:
    success, frame = cap.read()  # Čitanje slike sa kamere

    if not success:
        print('Greška')
        break
    frame = cv2.flip(frame, 1)
    frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    
    # Crtanje pravokutnika na slici
    cv2.rectangle(frame, (int(frame_width/2 - 100), 50), (int(frame_width/2 + 100), int(frame_height-50)), (0, 255, 0), 1)



    frame = detector.findHands(frame)  # Detekcija ruku na slici
    lmlist = detector.findPosition(frame, draw=False)  # Praćenje položaja ruku
    
    #Provjera otvorenosti šake
    if lmlist:
        finger_status = detector.findFingerStatus(lmlist)
        if finger_status < 3:  
            hand_open = 0   #Šaka zatvorena
        else:
            hand_open = 1 #Šaka otvorena

        current_x = lmlist[9][1]
        x_movement = current_x - prev_x #Smjer kretanja na x ravnini
        
        if x_movement > 0 and prev_x < frame_width / 2 +100 and current_x > frame_width / 2 +100:
            direction_speed += 1
            if direction_speed > 10:
                direction_speed = 10

        if x_movement < 0 and prev_x > frame_width / 2 -100 and current_x < frame_width / 2 -100:
            direction_speed -= 1
            if direction_speed < -10:
                direction_speed = -10

        prev_x = current_x #Spremanje trenutne pozicije
        
        #Slanje podataka na arduino samo ako je doslo do promjene vrijednosti
        if hand_open != prev_hand_open or direction_speed != prev_direction_speed:                
            print(f"Sending data to Arduino:{hand_open},{direction_speed}")                           
            data = f"{hand_open},{direction_speed}x"                                            
            ser.write(data.encode())                                                                                 
        prev_hand_open = hand_open                                                                            
        prev_direction_speed = direction_speed                                                             
       


    #Racunanje fps
    elapsed_time = time.time() - start_time
    fps= 1 / elapsed_time
    start_time = time.time()

    # Prikazivanje podataka u gornjem ljevom uglu
    cv2.rectangle(frame,(0,0),(100,70),(0,0,0),-1)
    cv2.putText(frame, f"FPS: {fps:.0f}", (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
    cv2.putText(frame, f"Speed: {direction_speed:.0f}", (10, 40), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
    if hand_open:
        cv2.putText(frame, f"ON", (10, 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
    else:
        cv2.putText(frame, f"OF", (10, 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)

    cv2.imshow("Camera", frame)  # Prikazivanje rezultirajuće slike

    #zatvaranje programa ako se pritisne tipka "q"
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()  #Isključivanje kamere
cv2.destroyAllWindows()  #Zatvaranje prozora za prikazivanje slike
data = f"{0},{0}x" #Slanje podataka kako bi se zaustavio rad motora
ser.write(data.encode())
