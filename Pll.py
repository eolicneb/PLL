#!/usr/bin/env python
# -*- coding: utf-8 -*
class Shaft():
  '''
  Esta clase encapsula las condiciones cinematicas del eje.
  Su funcion roll() recibe el torque y momento de inercia del eje y
  la duracion Dt [periodo de tiempo para diferencias finitas]
  para actualizar los valores cinematicos. La
  funcion cin() devuelve estos valores cinematios.
  '''
  def __init__(self, fase=0., vel=0., acel=0., **kwargs):
    # Variables cinamaticas:
    self.fase = fase
    self.vel = vel
    self.acel = acel
    self.minSpeed = kwargs.get('minSpeed', 0.0000001)

  def roll(self, torque=0., inercia=1., Dt=1.):
    self.acel = torque/inercia
    self.vel += self.acel*Dt
    if abs(self.vel) < self.minSpeed:
      self.vel = 0.
    self.fase += self.vel*Dt
    # print(f'{self.__repr__}: {self.instantanea()} con Dt={Dt}')

  def cin(self):
    return self.fase, self.vel, self.acel

  def set(self, estado=(0.,0.,0.)):
    self.fase, self.vel, self.acel = estado

  def setMinSpeed(self, minSpeed=0.0000001):
    self.minSpeed = minSpeed

  def getMinSpeed(self):
    return self.minSpeed

  def instantanea(self):
    return self.fase, self.vel, self.acel

class Torques():
  '''
  Clase abstracta para las distintas posibles acciones
  sobre un eje. La funcion roll() devuelve el valor de
  este torque.
  '''
  def __init__(self, **kwargs):
    self.torque = 0.
    self.eje = kwargs.get('eje', None)

  def setEje(self, eje):
    self.eje = eje

  def roll(self, **kwargs):
    return self.torque

class TorqueDirecto(Torques):
  '''
  Implementacion de la clase Torques que devuelve
  un torque proporcional a una signal de entrada
  segun una funcion de correspondencia definida
  por el usuario.
  '''
  def __init__(self, **kwargs):
    if 'funcionDeCorrespondencia' in kwargs:
      self.func = kwargs['funcionDeCorrespondencia']
    else:
      self.func = self.funcionBase
      self.tpu = kwargs.get('torquePorUnidad', 1.)

  def funcionBase(self, signal=0.):
    return self.tpu*signal

  def roll(self, **kwargs):
    signal = kwargs.get('signal', 0.)
    self.torque = self.func(signal)
    # print(f'{self.__repr__}: {self.torque} con señal {signal}')
    return self.torque

class CargaVelocidad(Torques):
  '''
  Implementa una clase Torques resistiva a la velocidad
  de giro del eje al que se le aplique.
  '''
  def __init__(self, **kwargs):
    self.tpu = kwargs.get('cargaPorRadianPorSeg', 1.)

  def roll(self, **kwargs):
    if not self.eje==None:
      _, vel, _ = self.eje.cin()
      self.torque = -1*self.tpu*vel
    else:
      self.torque = 0.
    return self.torque

class CargaAleatoria(Torques):
  ''' '''
  def __init__(self, **kwargs):
    self.Dt = kwargs.get('Dt', 1.0)
    self.rand_carga = kwargs.get('distAleatoria', (0.,1.))
    self.filtro = kwargs.get('filtro', 0.0)
    self.torque = 0.0

  def roll(self, **kwargs):
    from numpy.random import normal

    self.rand_carga = kwargs.get('distAleatoria', self.rand_carga)
    self.Dt=kwargs.get('Dt', self.Dt)

    if not self.eje == None:
      _, vel, _ = self.eje.cin()
      carga = -1*vel*self.Dt*normal(*self.rand_carga)**2
    else: 
      carga = 0.0

    self.torque = carga + self.filtro*(self.torque-carga)
    return self.torque

class Inercias():
  '''
  Clase abstracta para los momentos de inercia
  que lastran el giro de un eje.
  '''
  def __init__(self):
    self.inercia = 1.

  def roll(self, **kwargs):
    return self.inercia

class InerciaConstante(Inercias):
  '''
  La implementacion mas sencilla de la clase
  abstracta Inercias.
  '''
  def __init__(self, inercia=1.):
    self.inercia = inercia

class Encoder():
  '''
  Esta clase recibe un eje al que se acopla y devuelve
  los pulsos A, B y Z  que indican su posicion.
  '''
  def __init__(self, eje, res=1024):
    self.eje = eje # eje al que estÃƒÂ¡ fijado # eje al que estÃƒÂ¡ fijado
    self.res = res # resolucion
    self.A = 0
    self.B = 0
    self.Z = 0
    self.analizadores = []
    self.pulsos()

  def setAnalizador(self, analizador):
    self.analizadores.append(analizador)

  def getResolucion(self):
    return self.res

  def setResolucion(self, res=1024):
     self.res=res

  def pulsos(self):
    from numpy import pi
    fase, _, _ = self.eje.cin()
    ang = fase*self.res
    self.A = 1 if ang%(2*pi)<pi else 0
    self.B = 1 if (ang-pi/2)%(2*pi)<pi else 0
    self.Z = 1 if ang%(2*pi*self.res)<pi else 0

  def __call__(self, **kwargs):
    self.pulsos()
    for analizador in self.analizadores:
      analizador(origen=self, **kwargs)

  def getPulsos(self, **kwargs):
    shift = kwargs.get('shift', .0)
    return self.A+shift, self.B+shift, self.Z+shift

class MotorCombo():
  '''
  Implementa una estructura standard de eje+enconder con
  dos list que pasan instancias concretas de las clases
  Torques y Inercias.
  '''
  def __init__(self,
               res,
               torques,
               inercias):
    self.eje = Shaft()
    self.trq = torques #list de torques aplicados al eje
    self.ins = inercias #list de inercias aplicadas al eje
    self.enc = Encoder(eje=self.eje, res=res)
    self.load()

  def load(self):
    for t in self.trq: 
      t.setEje(self.eje)

  def roll(self, Dt=0.1, **kwargs):
    torqueTotal=.0
    for t in self.trq:
      torqueTotal += t.roll(eje=self.eje, **kwargs)
    inerciaTolal=.0
    for i in self.ins:
      inerciaTolal += i.roll(eje=self.eje, **kwargs)
    self.eje.roll(torqueTotal, inerciaTolal, Dt)
    kwargs['Dt']=Dt
    return self.enc(**kwargs)

  def getEncoder(self):
    return self.enc

  def getEstado(self):
    return self.eje.cin()

  def setEstado(self, *args):
    self.eje.set(*args)
    self.enc.pulsos()

class EncAnalizador():
  '''
  Clase abstracta para implementar los distintos procesos
  de analisis de signales basado en la salida de una
  pareja de encoders que se precisa sincronizar.
  El analisis debe implementarse en la funcion analisis().
  '''
  def __init__(self, enc1, enc2, **kwargs):
    self.enc1 = enc1
    self.enc2 = enc2
    self.output = 0.0
    self.time = 0.0
    self.Dt = kwargs.get('Dt', 1.0)
    self.maxOutput = kwargs.get('maxOuput', 100.)
    self.filtro = kwargs.get('filtro', 0.6)
    self.peso = kwargs.get('peso', 1.0)
    self.pid = kwargs.get('PID', (1., 0., 0.))
    self.prop = 0.0
    self.integral = 0.0
    self.derivada = 0.0
    # Esta es la variable q el usuario debe calcular en "analisis()"
    self.signal = 0.0 
    self.setup()
    self.initUsuario(**kwargs)

  def setEstado(self, registro):
    self.output=registro[0]
    self.time=registro[1]
    self.signal=registro[2]
    self.prop=registro[3]
    self.integral=registro[4]
    self.derivada=registro[5]
    self.setUsuario(registro[6])

  def getEstado(self):
    return [self.output,
            self.time,
            self.signal,
            self.prop,
            self.integral,
            self.derivada,
            self.getUsuario()]

  def setUsuario(self, *args):
    pass

  def getUsuario(self):
    return None

  def setup(self):
    self.enc1.setAnalizador(analizador=self.paraEncoder)
    self.enc2.setAnalizador(analizador=self.paraEncoder)

  def paraEncoder(self, **kwargs):
    print('En EncAnalizador.paraEncoder: {}'.format(kwargs))
    if 'time' in kwargs:
      print(' incoming time {} and own.time {}'.format(kwargs['time'], self.time))
      if kwargs['time'] > self.time:
        self.analisis(**kwargs)
        self.time = kwargs['time']
    else:
      self.analisis(**kwargs)

  def initUsuario(self, **kwargs):
    pass

  def __call__(self, **kwargs):
    self.Dt = kwargs.get('Dt', self.Dt)
    self.signal = clip(self.signal+self.filtro*(self.output-self.signal),
                        maximum=self.maxOutput)
    self.derivada = (self.signal-self.prop)/self.Dt
    self.integral = clip(self.signal*self.Dt+self.integral, maximum=self.maxOutput)
    self.prop = self.signal
    self.output = self.pid[0]*self.prop
    self.output += self.pid[1]*self.integral
    self.output += self.pid[2]*self.derivada
    self.output = self.peso*clip(self.output, maximum=self.maxOutput)
    return self.output

  def analisis(self, **kwargs):
    # This function must be overriden in derived classes
    print("Aca no deberias entrar")
    pass

class SeguirFrec(EncAnalizador):
  '''
  '''
  def initUsuario(self, **kwargs):
    self.flancoAnterior=[0.0, 0.0]
    self.pulsoAnterior=[1, 1]
    self.periodo1=1e128
    self.periodo2=1e128
    self.output=0.0
    self.pulsoReferencia=kwargs.get('referencia', 0)

  def getUsuario(self):
    bag = [self.flancoAnterior,
      self.pulsoAnterior,
      self.periodo1,
      self.periodo2,
      self.pulsoReferencia]
    return bag

  def setUsuario(self, registro):
    self.flancoAnterior=registro[0]
    self.pulsoAnterior=registro[1]
    self.periodo1=registro[2]
    self.periodo2=registro[3]
    self.pulsoReferencia=registro[4]

  def analisis(self,**kwargs):
    print('En SeguirFrec.analisis: {}'.format(kwargs))
    time = kwargs.get('time', self.time)
    ixP=self.pulsoReferencia
    p1 = self.enc1.getPulsos()
    p2 = self.enc2.getPulsos()
    flanco1 = p1[ixP]-self.pulsoAnterior[0]
    if flanco1 > 0:
      self.periodo1 = (time-self.flancoAnterior[0])*self.enc1.getResolucion()
      self.flancoAnterior[0] = time
    flanco2 = p2[ixP]-self.pulsoAnterior[1]
    if flanco2 > 0:
      self.periodo2 = (time-self.flancoAnterior[1])*self.enc2.getResolucion()
      self.flancoAnterior[1] = time
    self.pulsoAnterior[0] = p1[ixP]
    self.pulsoAnterior[1] = p2[ixP]
    self.signal = self.periodo2-self.periodo1

class PID(EncAnalizador):
  ''' '''
  def initUsuario(self, **kwargs):
    self.flancoAnterior = [.0, .0]
    self.pulsoAnterior = [1, 1]
    self.periodo = [10000.0, 10000.0]
    self.channel = kwargs.get('channel', 0)
    self.encoders = [self.enc1, self.enc2]

  def setUsuario(self, registro):
    self.flancoAnterior = registro[0]
    self.pulsoAnterior = registro[1]
    self.periodo = registro[2]

  def getUsuario(self):
    return [self.flancoAnterior,
            self.pulsoAnterior,
            self.periodo]

  def analisis(self,**kwargs):
    print('En PID.analisis: {}'.format(kwargs))
    time=kwargs.get('time', self.time)
    for n, enc in enumerate(self.encoders):
      p = enc.getPulsos()
      flanco = p[self.channel]-self.pulsoAnterior[n]
      self.pulsoAnterior[n] = p[self.channel]
      if flanco > 0:
        self.periodo[n] = (time-self.flancoAnterior[n])*enc.getResolucion()
        self.flancoAnterior[n] = time
    self.signal = self.periodo[1]-self.periodo[0]

class EnFase(EncAnalizador):
  ''' '''
  def initUsuario(self, **kwargs):
    self.flancoAnterior=[.0, .0]
    self.pulsoAnterior=[1, 1]
    self.periodo=[.0, .0]
    self.fase=[0., .0]
    self.encoders=[self.enc1, self.enc2]
    self.output=0.0

  def setEstado(self, registro):
    self.flancoAnterior=registro[0]
    self.pulsoAnterior=registro[1]
    self.periodo=registro[2]
    self.output=registro[3]
    self.filtro=registro[4]
    self.fase=registro[5]

  def getEstado(self):
    bag = [self.flancoAnterior,
           self.pulsoAnterior,
           self.periodo,
           self.output,
           self.filtro,
           self.fase]
    return bag

  def analisis(self, **kwargs):
    print('En EnFase.analisis: {}'.format(kwargs))
    time = kwargs.get('time', self.time)
    Dt = kwargs.get('Dt', 1./self.maxOutput)
    elOtro=[1, 0]
    desfase=self.output
    for n, enc in enumerate(self.encoders):
      p=enc.getPulsos()
      flanco = p[2]-self.pulsoAnterior[n]
      self.pulsoAnterior[n] = p[2]
      if flanco>0:
        self.periodo[n] = ((-1)**n)*(time-self.flancoAnterior[elOtro[n]])
      from math import fabs, copysign
      if fabs(desfase) < fabs(self.periodo[elOtro[n]]):
        desfase = desfase
      else:
        desfase = self.periodo[elOtro[n]]
      desfase=1./copysign(max(fabs(desfase),Dt),desfase)
    self.output = desfase+self.filtro*(self.output-desfase)

class FaseDetector(EncAnalizador):
  '''
  '''
  def initUsuario(self, **kwargs):
    time = kwargs.get('time', 0.0)
    self.flancoAnterior=[time, time]
    self.pulsoAnterior=[0, 0]
    self.periodo1=1.0
    self.periodo2=1.0
    self.desfase1=0.0
    self.desfase2=0.0
    self.pulsoReferencia = kwargs.get('referencia', 0)
    self.zero_safe = .00000001

  def getEstado(self):
    return [self.flancoAnterior,
            self.pulsoAnterior,
            self.periodo1,
            self.periodo2,
            self.desfase1,
            self.desfase2,
            self.pulsoReferencia]

  def setEstado(self, registro):
    self.flancoAnterior=registro[0]
    self.pulsoAnterior=registro[1]
    self.periodo1=registro[2]
    self.periodo2=registro[3]
    self.desfase1=registro[4]
    self.desfase2=registro[5]
    self.pulsoReferencia=registro[6]

  def analisis(self,**kwargs):
    print('En FaseDetector.analisis: {}'.format(kwargs))
    time = kwargs.get('time', self.time)
    ixP=self.pulsoReferencia
    p1 = self.enc1.getPulsos()
    p2 = self.enc2.getPulsos()
    lastTime = max(self.flancoAnterior)
    flanco1=p1[ixP]-self.pulsoAnterior[0]
    if flanco1>0:
      self.periodo1=time-self.flancoAnterior[0]
      self.desfase1=time-self.flancoAnterior[1]
      self.flancoAnterior[0]=time
    flanco2=p2[ixP]-self.pulsoAnterior[1]
    if flanco2>0:
      self.periodo2=time-self.flancoAnterior[1]
      self.desfase2=time-self.flancoAnterior[0]
      self.flancoAnterior[1]=time
    self.pulsoAnterior[0]=p1[ixP]
    self.pulsoAnterior[1]=p2[ixP]
    # Filtro pasabajos:
    self.desfase1 += self.zero_safe
    self.desfase2 += self.zero_safe
    self.signal = -self.desfase2**(-2)+self.desfase1**(-2)

class XORdetector(EncAnalizador):
    ''' '''
    def initUsuario(self, **kwargs):
      self.pulsoReferencia=kwargs.get('referencia', 0)

    def getEstado(self):
      return [self.pulsoReferencia]

    def setEstado(self, registro):
      self.pulsoReferencia=registro[0]

    def analisis(self, **kwargs):
      print('En XORdetector.analisis: {}'.format(kwargs))
      ixP = self.pulsoReferencia
      p1 = self.enc1.getPulsos()
      p2 = self.enc2.getPulsos()
      self.signal = -1 if p1[ixP]==p2[ixP] else 1

# class MagnticPhase(EncAnalizador):
#   """
#   Subtracts the square of the distance of the later to the previous
#   other encoder's pulse as if magnetically atracted.
#   """
#   def initUsuario(self, **kwargs):
#     self.channel = kwargs('channel', 2)

  

class Signal():
  '''
  Una clase que compone todos los analizadores utilizados
  y devuelve una unica señal para insertar en el motor
  dirigido. Toma de entrada una lista de objetos EncAnalizardor
  '''
  def __init__(self, analizadores, **kwargs):
    self.lista=analizadores
    self.signal=kwargs.get('signal', 0.0)
    self.delta=0.0
    self.MAX_DELTA=kwargs.get('MAX_DELTA', False)

  def __call__(self, **kwargs):
    Dt = kwargs.get('Dt', 1.)
    delta = 0.0
    for am in self.lista:
      delta += am(**kwargs)
    self.delta = clip(delta, maximum=self.MAX_DELTA*Dt) if self.MAX_DELTA else delta
    self.signal += self.delta*Dt
    self.signal = max((self.signal, 0.))
    # print(self.delta*Dt, self.signal)
    return self.signal

  def getEstado(self):
    bag = [ am.getEstado() for am in self.lista ]
    return [ bag, self.signal, self.delta ]

  def setEstado(self, registro):
    bag, self.signal, self.delta = registro
    for am, reg in zip(self.lista, bag):
      am.setEstado(reg)

def clip(o, maximum=False, minimum=False):
  from math import copysign, fabs
  ret = o
  if maximum:
    ret = copysign(min([fabs(ret), fabs(maximum)]), ret)
  if minimum:
    ret = copysign(max([fabs(ret), fabs(minimum)]), ret)
  return ret

def pickle_save(pickle_file, cosas):
  from pickle import dump
  with open(pickle_file, 'wb') as f:
    dump(cosas, f)

def pickle_load(pickle_file):
  from pickle import load
  with open(pickle_file, 'rb') as f:
    cosas=load(f)
    return cosas

# Final de las declaraciones de Clases y Funciones

if __name__=='__main__':

  pickle_file = "registro005.pkl" #
  continuar=0
  Dt=.01
  MAXT=2

  Res1=16
  Res2=16

  signal=1.
  MAX_DELTA=100.

  # PID on channel A
  K1 = 1.e1
  PID1 = (.05, 0., .0)
  filtro1 = 0.0
  # FaseDetector on channel C
  K2=0.e2
  PID2=(1.,0.,0.01)
  filtro2 = 0.1
  # EnFase on channel C
  K3=0.e4
  PID3=(1.,.0,.0)
  filtro3 = 0.1
  # PID on channel C
  K4=0.e1
  PID4=(1.,.1,0.)
  filtro4 = 0.6

  def target(time,t=.2,b=.0,c=.1,T=200.):
    from math import sin
    return t*(1+b*time**2/MAXT**2)+c*sin(time*6.28/T)**2

  mot1 = MotorCombo(Res1,
                    [ TorqueDirecto(),
                      CargaVelocidad()],
                    [InerciaConstante()])

  mot2 = MotorCombo(Res2,
                    [ TorqueDirecto(),
                      CargaVelocidad(),
                      CargaAleatoria(distAleatoria=(2.,1.), filtro=0.6)],
                    [InerciaConstante()])

  ana1 = PID(enc1=mot1.getEncoder(),
             enc2=mot2.getEncoder(),
             peso=K1,
             filtro=filtro1,
             PID=PID1,
             Dt=Dt)

  ana2 = FaseDetector(enc1=mot1.getEncoder(),
                      enc2=mot2.getEncoder(),
                      peso=K2,
                      PID=PID2,
                      referencia=2,
                      Dt=Dt)

  # ana3 = EnFase(enc1=mot1.getEncoder(),
  #               enc2=mot2.getEncoder(),
  #               peso=K3,
  #               PID=PID3,
  #               filtro=filtro3,
  #               Dt=Dt)

  # ana4 = PID(enc1=mot1.getEncoder(),
  #            enc2=mot2.getEncoder(),
  #            channel=2,
  #            peso=K2,
  #            filtro=filtro2,
  #            PID=PID2,
  #            Dt=Dt)

  sen=Signal([ana1], # , ana2, ana3, ana4
              MAX_DELTA=MAX_DELTA,
              signal=signal)

  # En la CajaDeZapatos se ponen los objetos
  # cuyos estados se quieran guardar y recuperar.
  CajaDeZapatos=[mot1, mot2, sen]

  time = 0.0

  if continuar:
    cosas=pickle_load(pickle_file)
    for donde, que in zip(CajaDeZapatos, cosas[0]):
      donde.setEstado(que)
    time = cosas[1]
    MAXT += time

  def step(time):
    time+=Dt
    kwargs={'time':time, 'Dt':Dt}
    trg = target(time)
    reg = sen(**kwargs)
    mot1.roll(signal=trg, **kwargs)
    mot2.roll(signal=reg, **kwargs)
    enc1 = mot1.getEncoder().getPulsos(shift=.1)
    enc2 = mot2.getEncoder().getPulsos()
    integral = ana1.integral
    derivat = ana1.derivada
    prop = ana1.prop
    return time, (reg, integral, derivat, prop, trg, enc1, enc2)

  X=[]
  integral=[]; derivat = []; prop = []
  reg=[]; trg=[]
  enc1=[]; enc2=[]
  lists = (reg, integral, derivat, prop, trg, enc1, enc2)

  while time<MAXT :
    time, new_values = step(time)
    X.append(time)
    for l_, val in zip(lists, new_values):
      l_.append(val)

  print('Finales: ')
  print('Eje 1: ', mot1.getEstado())
  print('Eje 2: ', mot2.getEstado())
  print('Time : ', time)
  print('Signal: ', sen.signal)

  cosas = [de.getEstado() for de in CajaDeZapatos]
  cosas += [time]
  pickle_save(pickle_file, cosas)

  import matplotlib.pyplot as plt
  from matplotlib.animation import FuncAnimation as FuncAn

  fig, (ax1, ax2, ax3, ax4, _) = plt.subplots(5, 1, sharex=True)
  ax2.set_ylim(-MAX_DELTA*1.1,MAX_DELTA*1.1)
  ax5 = fig.add_subplot(529, projection='polar')
  ax6 = fig.add_subplot(5,2,10, projection='polar')
  A1, B1, Z1 = zip(*enc1)
  A2, B2, Z2 = zip(*enc2)
  ln11, ln12 = ax1.plot(X, A1, X, A2)
  ln21, ln22, ln23 = ax2.plot(X, integral, X, derivat, X, prop)
  ln31, ln32 = ax3.plot(X, trg, X, reg)
  ln41, ln42 = ax4.plot(X, Z1, X, Z2)
  ln5, = ax5.plot((mot1.eje.fase,)*2, (.1,1.), 'tab:blue')
  ln6, = ax6.plot((mot2.eje.fase,)*2, (.1,1.), 'tab:orange')

  class Updater():
    def __init__(self, time, step, batch=20):
      self.time = time
      self.step = step # Function to get new values
      self.batch = batch

    def update(self, i):
      for _ in range(self.batch):
        self.time, new_values = self.step(self.time)
        for l_, val in zip(lists, new_values):
          l_[:] = l_[1:]+[val]
      fase1, fase2 = mot1.eje.fase, mot2.eje.fase

      A1, B1, Z1 = zip(*enc1)
      A2, B2, Z2 = zip(*enc2)
      ln11.set_data(X, A1)
      ln12.set_data(X, A2)
      ln21.set_data(X, integral)
      ln22.set_data(X, derivat)
      ln23.set_data(X, prop)
      ln31.set_data(X, trg)
      ln32.set_data(X, reg)
      # ax3.autoscale(enable=True, axis='y')
      ax3.set_ylim(-.2+min(min(trg),min(reg)),.2+max(max(trg),max(reg)))
      ln41.set_data(X, Z1)
      ln42.set_data(X, Z2)
      ln5.set_data((fase1,)*2, (.1,1.))
      ln6.set_data((fase2,)*2, (.1,1.)) 

      return (ln11, ln12, ln21, ln22, ln23, ln31, ln32, ln41, ln42, ln5, ln6)
  
  updater = Updater(time, step, 30)

  func_an = FuncAn(fig, updater.update, frames=range(100), interval=20, blit=True)

  plt.show()
