import sys
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QLabel, QWidget, QLineEdit, QHBoxLayout, QVBoxLayout, QPushButton,QGroupBox
from PyQt5.QtGui import QDoubleValidator
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from stl import mesh
from numpy import array
import numpy as np
from math import pi,cos,sin

###### Crie suas funções de translação, rotação, criação de referenciais, plotagem de setas e qualquer outra função que precisar

def set_axes_equal(ax):
    #Make axes of 3D plot have equal scale so that spheres appear as spheres,
    #cubes as cubes, etc..  This is one possible solution to Matplotlib's
    #ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    #Input
    #  ax: a matplotlib axis, e.g., as output from plt.gca().

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
    
def draw_arrows(point,base,axis,length=1.5):
    # The object base is a matrix, where each column represents the vector
    # of one of the axis, written in homogeneous coordinates (ax,ay,az,0)
    # Plot vector of x-axis
    axis.quiver(point[0],point[1],point[2],base[0,0],base[1,0],base[2,0],color='red',pivot='tail',  length=length)
    # Plot vector of y-axis
    axis.quiver(point[0],point[1],point[2],base[0,1],base[1,1],base[2,1],color='green',pivot='tail',  length=length)
    # Plot vector of z-axis
    axis.quiver(point[0],point[1],point[2],base[0,2],base[1,2],base[2,2],color='blue',pivot='tail',  length=length)

    return axis

def z_rotation(angle):
    rotation_matrix=np.array([[cos(angle),-sin(angle),0,0],[sin(angle),cos(angle),0,0],[0,0,1,0],[0,0,0,1]])
    return rotation_matrix

def x_rotation(angle):
    rotation_matrix=np.array([[1,0,0,0],[0, cos(angle),-sin(angle),0],[0, sin(angle), cos(angle),0],[0,0,0,1]])
    return rotation_matrix

def y_rotation(angle):
    rotation_matrix=np.array([[cos(angle),0, sin(angle),0],[0,1,0,0],[-sin(angle), 0, cos(angle),0],[0,0,0,1]])
    return rotation_matrix

def move (dx,dy,dz):
    T = np.eye(4)
    T[0,-1] = dx
    T[1,-1] = dy
    T[2,-1] = dz
    return T

# ---------------------------------------------------------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        #definindo as variaveis
        self.set_variables()
        #Ajustando a tela    
        self.setWindowTitle("Grid Layout")
        self.setGeometry(100, 100, 1280, 720)
        self.setup_ui()

    def set_variables(self):      
        # WORLD 
        e1 = np.array([[1],[0],[0],[0]]) # X
        e2 = np.array([[0],[1],[0],[0]]) # Y
        e3 = np.array([[0],[0],[1],[0]]) # Z
        self.base = np.hstack((e1,e2,e3))
        
        # OBJ
        your_mesh = mesh.Mesh.from_file('fox.stl')
        # Get the x, y, z coordinates contained in the mesh structure
        x = your_mesh.x.flatten()
        y = your_mesh.y.flatten()
        z = your_mesh.z.flatten()
        self.objeto_original = array([
            (x.T - x.T[-1])/10,
            (y.T - y.T[-1])/10,
            (z.T - z.T[-1])/10,
            np.ones(x.size)
            ])
        self.objeto = self.objeto_original
        
        # CAM
        self.cam_original = np.eye(4)
        
        T = move(-10., 2, 4.9)
        Ry = y_rotation(pi/2)
        Rx = x_rotation(-pi/2)
        self.M_cam = T @ Rx @ Ry
        
        self.cam = self.M_cam @ self.cam_original
        self.px_base = 1280  
        self.px_altura = 720 
        self.dist_foc = 20 
        self.stheta = 0 
        self.ox = self.px_base/2 
        self.oy = self.px_altura/2 
        self.ccd = [36,24] 
        self.projection_matrix = []
        
    def setup_ui(self):
        # Criar o layout de grade
        grid_layout = QGridLayout()

        # Criar os widgets
        line_edit_widget1 = self.create_world_widget("Ref mundo")
        line_edit_widget2  = self.create_cam_widget("Ref camera")
        line_edit_widget3  = self.create_intrinsic_widget("params instr")

        self.canvas = self.create_matplotlib_canvas()

        # Adicionar os widgets ao layout de grade
        grid_layout.addWidget(line_edit_widget1, 0, 0)
        grid_layout.addWidget(line_edit_widget2, 0, 1)
        grid_layout.addWidget(line_edit_widget3, 0, 2)
        grid_layout.addWidget(self.canvas, 1, 0, 1, 3)

          # Criar um widget para agrupar o botão de reset
        reset_widget = QWidget()
        reset_layout = QHBoxLayout()
        reset_widget.setLayout(reset_layout)

        # Criar o botão de reset vermelho
        reset_button = QPushButton("Reset")
        reset_button.setFixedSize(50, 30)  # Define um tamanho fixo para o botão (largura: 50 pixels, altura: 30 pixels)
        style_sheet = """
            QPushButton {
                color : white ;
                background: rgba(255, 127, 130,128);
                font: inherit;
                border-radius: 5px;
                line-height: 1;
            }
        """
        reset_button.setStyleSheet(style_sheet)
        reset_button.clicked.connect(self.reset_canvas)

        # Adicionar o botão de reset ao layout
        reset_layout.addWidget(reset_button)

        # Adicionar o widget de reset ao layout de grade
        grid_layout.addWidget(reset_widget, 2, 0, 1, 3)

        # Criar um widget central e definir o layout de grade como seu layout
        central_widget = QWidget()
        central_widget.setLayout(grid_layout)
        
        # Definir o widget central na janela principal
        self.setCentralWidget(central_widget)

    def create_intrinsic_widget(self, title):
        # Criar um widget para agrupar os QLineEdit
        line_edit_widget = QGroupBox(title)
        line_edit_layout = QVBoxLayout()
        line_edit_widget.setLayout(line_edit_layout)

        # Criar um layout de grade para dividir os QLineEdit em 3 colunas
        grid_layout = QGridLayout()

        line_edits = []
        labels = ['n_pixels_base:', 'n_pixels_altura:', 'ccd_x:', 'ccd_y:', 'dist_focal:', 'sθ:']  # Texto a ser exibido antes de cada QLineEdit

        # Adicionar widgets QLineEdit com caixa de texto ao layout de grade
        for i in range(1, 7):
            line_edit = QLineEdit()
            label = QLabel(labels[i-1])
            validator = QDoubleValidator()  # Validador numérico
            line_edit.setValidator(validator)  # Aplicar o validador ao QLineEdit
            grid_layout.addWidget(label, (i-1)//2, 2*((i-1)%2))
            grid_layout.addWidget(line_edit, (i-1)//2, 2*((i-1)%2) + 1)
            line_edits.append(line_edit)

        # Criar o botão de atualização
        update_button = QPushButton("Atualizar")

        ##### Você deverá criar, no espaço reservado ao final, a função self.update_params_intrinsc ou outra que você queira 
        # Conectar a função de atualização aos sinais de clique do botão
        update_button.clicked.connect(lambda: self.update_params_intrinsc(line_edits))

        # Adicionar os widgets ao layout do widget line_edit_widget
        line_edit_layout.addLayout(grid_layout)
        line_edit_layout.addWidget(update_button)

        # Retornar o widget e a lista de caixas de texto
        return line_edit_widget
    
    def create_world_widget(self, title):
        # Criar um widget para agrupar os QLineEdit
        line_edit_widget = QGroupBox(title)
        line_edit_layout = QVBoxLayout()
        line_edit_widget.setLayout(line_edit_layout)

        # Criar um layout de grade para dividir os QLineEdit em 3 colunas
        grid_layout = QGridLayout()

        line_edits = []
        labels = ['X(move):', 'X(angle):', 'Y(move):', 'Y(angle):', 'Z(move):', 'Z(angle):']  # Texto a ser exibido antes de cada QLineEdit

        # Adicionar widgets QLineEdit com caixa de texto ao layout de grade
        for i in range(1, 7):
            line_edit = QLineEdit()
            label = QLabel(labels[i-1])
            validator = QDoubleValidator()  # Validador numérico
            line_edit.setValidator(validator)  # Aplicar o validador ao QLineEdit
            grid_layout.addWidget(label, (i-1)//2, 2*((i-1)%2))
            grid_layout.addWidget(line_edit, (i-1)//2, 2*((i-1)%2) + 1)
            line_edits.append(line_edit)

        # Criar o botão de atualização
        update_button = QPushButton("Atualizar")

        ##### Você deverá criar, no espaço reservado ao final, a função self.update_world ou outra que você queira 
        # Conectar a função de atualização aos sinais de clique do botão
        update_button.clicked.connect(lambda: self.update_world(line_edits))

        # Adicionar os widgets ao layout do widget line_edit_widget
        line_edit_layout.addLayout(grid_layout)
        line_edit_layout.addWidget(update_button)

        # Retornar o widget e a lista de caixas de texto
        return line_edit_widget

    def create_cam_widget(self, title):
        # Criar um widget para agrupar os QLineEdit
        line_edit_widget = QGroupBox(title)
        line_edit_layout = QVBoxLayout()
        line_edit_widget.setLayout(line_edit_layout)

        # Criar um layout de grade para dividir os QLineEdit em 3 colunas
        grid_layout = QGridLayout()

        line_edits = []
        labels = ['X(move):', 'X(angle):', 'Y(move):', 'Y(angle):', 'Z(move):', 'Z(angle):']  # Texto a ser exibido antes de cada QLineEdit

        # Adicionar widgets QLineEdit com caixa de texto ao layout de grade
        for i in range(1, 7):
            line_edit = QLineEdit()
            label = QLabel(labels[i-1])
            validator = QDoubleValidator()  # Validador numérico
            line_edit.setValidator(validator)  # Aplicar o validador ao QLineEdit
            grid_layout.addWidget(label, (i-1)//2, 2*((i-1)%2))
            grid_layout.addWidget(line_edit, (i-1)//2, 2*((i-1)%2) + 1)
            line_edits.append(line_edit)

        # Criar o botão de atualização
        update_button = QPushButton("Atualizar")

        ##### Você deverá criar, no espaço reservado ao final, a função self.update_cam ou outra que você queira 
        # Conectar a função de atualização aos sinais de clique do botão
        update_button.clicked.connect(lambda: self.update_cam(line_edits))

        # Adicionar os widgets ao layout do widget line_edit_widget
        line_edit_layout.addLayout(grid_layout)
        line_edit_layout.addWidget(update_button)

        # Retornar o widget e a lista de caixas de texto
        return line_edit_widget

    def create_matplotlib_canvas(self):
        # Criar um widget para exibir os gráficos do Matplotlib
        canvas_widget = QWidget()
        canvas_layout = QHBoxLayout()
        canvas_widget.setLayout(canvas_layout)

        # Criar um objeto FigureCanvas para exibir o gráfico 2D
        self.fig1, self.ax1 = plt.subplots()
        self.ax1.set_title("Imagem")
        self.canvas1 = FigureCanvas(self.fig1)

        ##### limites do eixo X
        self.ax1.set_xlim([0, self.px_base])
        
        ##### limites do eixo Y
        self.ax1.set_ylim([self.px_altura, 0])
        
        ##### Você deverá criar a função de projeção 
        object_2d = self.projection_2d()

        ##### Falta plotar o object_2d que retornou da projeção
        self.ax1.plot(object_2d[0],object_2d[1])
        self.ax1.grid('True')
        self.ax1.set_aspect('equal')  
        canvas_layout.addWidget(self.canvas1)

        # Criar um objeto FigureCanvas para exibir o gráfico 3D
        self.fig2 = plt.figure()
        self.ax2 = self.fig2.add_subplot(111, projection='3d')
        
        ##### Falta plotar o seu objeto 3D e os referenciais da câmera e do mundo
        self.ax2.plot(self.objeto[0], self.objeto[1], self.objeto[2])
        point = np.array([[0],[0],[0],[1]])
        draw_arrows(point, self.base, self.ax2, 1.5)
        draw_arrows(self.cam[:,-1], self.cam[:,0:3], self.ax2, 1)
        
        # manter aspect ratio
        set_axes_equal(self.ax2)
        
        self.canvas2 = FigureCanvas(self.fig2)
        canvas_layout.addWidget(self.canvas2)

        # Retornar o widget de canvas
        return canvas_widget


    ##### Você deverá criar as suas funções aqui
    
    def update_params_intrinsc(self, line_edits: list[QLineEdit]):
        if line_edits[0].displayText():  # n_pixels_base
            self.px_base = int(line_edits[0].displayText())
            print("Update px_base")
        
        if line_edits[1].displayText():  # n_pixels_altura
            self.px_altura = int(line_edits[1].displayText())
            print("Update px_altura")
        
        if line_edits[2].displayText():  # ccd_x
            self.ccd = (float(line_edits[2].displayText()), self.ccd[1])
            print("Update ccd_x")
        
        if line_edits[3].displayText():  # ccd_y
            self.ccd = (self.ccd[0], float(line_edits[3].displayText()))
            print("Update ccd_y")
        
        if line_edits[4].displayText():  # dist_foc
            self.dist_foc = float(line_edits[4].displayText())
            print("Update distancia focal")
            
        if line_edits[5].displayText():  # stheta
            self.stheta = float(line_edits[5].displayText())
            print("Update stheta")
        
        self.update_canvas()
        return 

    def update_world(self, line_edits: list[QLineEdit]):
        x_trans, y_trans, z_trans = 0, 0, 0
        if line_edits[0].displayText():  # X trans
            x_trans = float(line_edits[0].displayText())
            T = move(x_trans,0,0)
            print(f"transalte {x_trans} in x (world ref)")
            self.cam = T @ self.cam
            self.M_cam = T @ self.M_cam
        
        if line_edits[1].displayText():  # X rot
            x_rot = float(line_edits[1].displayText())
            R = x_rotation(x_rot * (pi/180))
            print(f"rotate {x_rot} in x (world ref)")
            self.cam = R @ self.cam
            self.M_cam = R @ self.M_cam
        
        if line_edits[2].displayText():  # Y trans
            y_trans = float(line_edits[2].displayText())
            T = move(0,y_trans,0)
            print(f"transalte {y_trans} in y (world ref)")
            self.cam = T @ self.cam
            self.M_cam = T @ self.M_cam
        
        if line_edits[3].displayText():  # Y rot
            y_rot = float(line_edits[3].displayText())
            R = y_rotation(y_rot * (pi/180))
            print(f"rotate {y_rot} in y (world ref)")
            self.cam = R @ self.cam
            self.M_cam = R @ self.M_cam
        
        if line_edits[4].displayText():  # Z trans
            z_trans = float(line_edits[4].displayText())
            T = move(0,0, z_trans)
            print(f"transalte {z_trans} in z (world ref)")
            self.cam = T @ self.cam
            self.M_cam = T @ self.M_cam
            
        if line_edits[5].displayText():  # Z rot
            z_rot = float(line_edits[5].displayText())
            R = z_rotation(z_rot * (pi/180))
            print(f"rotate {z_rot} in z (world ref)")
            self.cam = R @ self.cam
            self.M_cam = R @ self.M_cam
            
        self.update_canvas()
        return

    def update_cam(self, line_edits: list[QLineEdit]):
        if line_edits[0].displayText():  # X trans
            x_trans = float(line_edits[0].displayText())
            T = move(x_trans, 0, 0)
            print(f"transalte {x_trans} in x (cam ref)")
            cam_aux = np.linalg.inv(self.M_cam) @ self.cam
            self.cam = self.M_cam @ T @ cam_aux
            self.M_cam = self.M_cam @ T
            
        if line_edits[1].displayText():  # X rot
            x_rot = float(line_edits[1].displayText())
            R = x_rotation(x_rot * (pi/180))
            print(f"rotate {x_rot} in x (cam ref)")
            cam_aux = np.linalg.inv(self.M_cam) @ self.cam
            self.cam = self.M_cam @ R @ cam_aux
            self.M_cam = self.M_cam @ R
        
        if line_edits[2].displayText():  # Y trans
            y_trans = float(line_edits[2].displayText())
            T = move(0, y_trans, 0)
            print(f"transalte {y_trans} in y (cam ref)")
            cam_aux = np.linalg.inv(self.M_cam) @ self.cam
            self.cam = self.M_cam @ T @ cam_aux
            self.M_cam = self.M_cam @ T
            
        if line_edits[3].displayText():  # Y rot
            y_rot = float(line_edits[3].displayText())
            R = y_rotation(y_rot * (pi/180))
            print(f"rotate {y_rot} in y (cam ref)")
            cam_aux = np.linalg.inv(self.M_cam) @ self.cam
            self.cam = self.M_cam @ R @ cam_aux
            self.M_cam = self.M_cam @ R
        
        if line_edits[4].displayText():  # Z trans
            ztrans = float(line_edits[4].displayText())
            T = move(0, 0, ztrans)
            print(f"transalte {ztrans} in z (cam ref)")
            cam_aux = np.linalg.inv(self.M_cam) @ self.cam
            self.cam = self.M_cam @ T @ cam_aux
            self.M_cam = self.M_cam @ T
            
        if line_edits[5].displayText():  # Z rot
            z_rot = float(line_edits[5].displayText())
            R = z_rotation(z_rot * (pi/180))
            print(f"rotate {z_rot} in z (cam ref)")
            cam_aux = np.linalg.inv(self.M_cam) @ self.cam
            self.cam = self.M_cam @ R @ cam_aux
            self.M_cam = self.M_cam @ R
        
        self.update_canvas()
        return 
    
    # Retorna o objeto em 2d projetado na camera
    def projection_2d(self):
        K = self.generate_intrinsic_params_matrix()
        P_can = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
        G = np.linalg.inv(self.M_cam)
        self.projection_matrix = K @ P_can @ G
        objeto_2d = self.projection_matrix @ self.objeto
        imagem = objeto_2d/objeto_2d[2]
        return imagem
      
    def generate_intrinsic_params_matrix(self):
        K = np.eye(3)

        K[0][0] = self.dist_foc * (self.px_base / self.ccd[0])
        K[0][1] = self.dist_foc * self.stheta      
        K[0][2] = self.ox     

        K[1][1] = self.dist_foc * (self.px_altura / self.ccd[1])
        K[1][2] = self.oy
        
        return K
    

    def update_canvas(self):
        print("Reloading canvas")
  
        # Clear and recreate 2D plot
        self.fig1.clear()
        self.ax1 = self.fig1.add_subplot(111)
        self.ax1.set_title("Imagem")
        self.ax1.set_xlim([0, self.px_base])
        self.ax1.set_ylim([self.px_altura, 0])
        self.ax1.set_aspect('equal')
        self.ax1.grid(True)

        object_2d = self.projection_2d()
        self.ax1.plot(object_2d[0], object_2d[1])

        self.canvas1.draw()

        # Clear and recreate 3D plot
        self.fig2.clear()
        self.ax2 = self.fig2.add_subplot(111, projection='3d')

        self.ax2.plot(self.objeto[0], self.objeto[1], self.objeto[2])
        point = np.array([[0], [0], [0], [1]])
        draw_arrows(point, self.base, self.ax2, 1.5)
        draw_arrows(self.cam[:, -1], self.cam[:, 0:3], self.ax2, 1)

        set_axes_equal(self.ax2)

        self.canvas2.draw()
        return 
    
    def reset_canvas(self):
        print("Reseting all variables")
        self.set_variables()
        self.update_canvas()
        return
    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
