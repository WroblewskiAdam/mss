from vpython import scene, box, rate, vector, set_browser

# --- Spróbuj najpierw tego ---
# set_browser('server')
# --- Jeśli to nie zadziała, a masz pulpit na RPi, spróbuj tego ---
# set_browser('none')

scene.title = "Prosty Test VPython"
scene.width = 600
scene.height = 400

klocek = box(pos=vector(0,0,0), size=vector(1,1,1), color=vector(1,0,0)) # Czerwony klocek

i = 0
while True:
    rate(30) # Ogranicz do 30 klatek na sekundę
    klocek.rotate(angle=0.01, axis=vector(0,1,0)) # Obracaj wokół osi Y
    i += 1
    if i % 100 == 0:
        print(f"Minimalny test działa, iteracja: {i}")
