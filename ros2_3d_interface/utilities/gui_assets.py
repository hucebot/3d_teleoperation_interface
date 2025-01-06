import pyglet

class Button:
    def __init__(self, label, x, y, width, height, on_click):
        self.label = label
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.on_click = on_click

    def draw(self):
        # Dibujar el botón como un rectángulo
        pyglet.graphics.draw(4, pyglet.gl.GL_QUADS, ('v2f', [
            self.x, self.y,
            self.x + self.width, self.y,
            self.x + self.width, self.y + self.height,
            self.x, self.y + self.height
        ]))
        # Dibujar el texto
        label = pyglet.text.Label(
            self.label, font_size=12, x=self.x + self.width // 2, y=self.y + self.height // 2,
            anchor_x='center', anchor_y='center'
        )
        label.draw()

    def check_click(self, x, y):
        if self.x <= x <= self.x + self.width and self.y <= y <= self.y + self.height:
            self.on_click()