

class Panel:

    def __init__(self, conn):
        self.conn = conn
        self.canvas = self.conn.ui.stock_canvas
        self.panel = self.canvas.add_panel()
        self.position_panel()
        self.texts = {}

    def init_panel(self, status):
        y_pos = 10
        for key in status:
            text = key + ": " + str(status[key])
            self.texts[key] = self.add_text(text, y_pos)
            y_pos = y_pos + 20

    def update_panel(self, status):
        for key in status:
            self.texts[key].content = key + ": " + str(status[key])

    def add_text(self, text, position):
        text = self.panel.add_text(text)
        text.rect_transform.size = (200., 30.)
        text.rect_transform.position = (10, position)
        text.color = (1, 1, 1)
        text.size = 18
        return text

    def position_panel(self):
        rect = self.panel.rect_transform
        rect.size = (200, 200)
        screen_size = self.canvas.rect_transform.size
        rect.position = (150. - (screen_size[0]/2.0), 0)
