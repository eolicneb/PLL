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
    self.minSpeed = kwargs['minSpeed'] if ('minSpeed' in kwargs) else 0.001

  def roll(self, torque=0., inercia=1., Dt=1.):
    self.acel = torque/inercia
    self.vel += self.acel*Dt
    if abs(self.vel)<self.minSpeed: self.vel = 0.
    self.fase += self.vel*Dt

  def cin(self):
    return self.fase, self.vel, self.acel

  def set(self, estado=(0.,0.,0.)):
    self.fase, self.vel, self.acel = estado

  def setMinSpeed(self, minSpeed=0.001):
    self.minSpeed = minSpeed

  def getMinSpeed(self):
    return self.minSpeed

  def instantanea(self):
    return [self.fase, self.vel, self.acel]

class Torques():
  '''
  Clase abstracta para las distintas posibles acciones
  sobre un eje. La funcion roll() devuelve el valor de
  este torque.
  '''
  def __init__(self, **kwargs):
    self.torque = 0.
    self.eje = kwargs['eje'] if 'eje' in kwargs else None

  def setEje(self, eje):
    self.eje = eje

  def roll(self, **kwargs):
    return self.torque

class TorqueDirecto(Torques):
  '''
  Implementacion de la clase Torques que devuelve
  un torque proporcional a una senal de entrada
  segun una funcion de correspondencia definida
  por el usuario.
  '''
  def __init__(self, **kwargs):
    if 'funcionDeCorrespondencia' in kwargs:
      self.func = kwargs['funcionDeCorrespondencia']
    else:
      self.func = self.funcionBase
      self.tpu = kwargs['torquePorUnidad'] if ('torquePorUnidad' in kwargs) else 1.

  def funcionBase(self, senal=0.):
    return self.tpu*senal

  def roll(self, **kwargs):
    senal = kwargs['senal'] if ('senal' in kwargs) else 0.
    self.torque = self.func(senal)
    return self.torque

class CargaVelocidad(Torques):
  '''
  Implementa una clase Torques resistiva a la velocidad
  de giro del eje al que se le aplique.
  '''
  def __init__(self, **kwargs):
    self.tpu = kwargs['cargaPorRadianPorSeg'] if ('cargaPorRadianPorSeg' in kwargs) else 1.

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
    self.Dt=kwargs['Dt'] if 'Dt' in kwargs else 1.0
    self.rand_carga=kwargs['distAleatoria'] if 'distAleatoria' in kwargs else (0.,1.)
    self.filtro=kwargs['filtro'] if 'filtro' in kwargs else 0.0
    self.torque=0.0

  def roll(self, **kwargs):
    self.rand_carga=kwargs['distAleatoria'] if 'distAleatoria' in kwargs else self.rand_carga
    self.Dt=kwargs['Dt'] if 'Dt' in kwargs else self.Dt
    if not self.eje==None:
      _, vel, _ = self.eje.cin()
      from numpy.random import normal
      carga = -1*vel*self.Dt*normal(*self.rand_carga)**2
    else: carga=0.0
    self.torque=carga+self.filtro*(self.torque-carga)
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
    self.analizador = []
    self.pulsos()

  def setAnalizador(self, analizador):
    self.analizador.append(analizador)

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
    if len(self.analizador)>0:
      for n in self.analizador:
        n(origen=self, **kwargs)

  def getPulsos(self, **kwargs):
    shift = kwargs['shift'] if 'shift' in kwargs else .0
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
    for t in self.trq: t.setEje(self.eje)

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
  de analisis de senales basado en la salida de una
  pareja de encoders que se precisan sincronizar.
  El analisis debe implementarse en la funcion analisis().
  '''
  def __init__(self, enc1, enc2, **kwargs):
    self.output = 0.0
    self.Dt = kwargs['Dt'] if 'Dt' in kwargs else 1.0
    self.maxOutput = kwargs['maxOuput'] if 'maxOuput' in kwargs else 10.
    self.enc1 = enc1
    self.enc2 = enc2
    self.time = 0.0
    self.filtro = kwargs['filtro'] if 'filtro' in kwargs else 0.6
    self.peso = kwargs['peso'] if 'peso' in kwargs else 1.0
    self.pid = kwargs['PID'] if 'PID' in kwargs else (1., 0., 0.)
    self.prop = 0.0
    self.integral = 0.0
    self.derivada = 0.0
    self.senal=0.0 # Esta es la variable q el usuario debe calcular en "analisis()"
    self.setup()
    self.initUsuario(**kwargs)

  def setEstado(self, registro):
    self.output=registro[0]
    self.time=registro[1]
    self.senal=registro[2]
    self.prop=registro[3]
    self.integral=registro[4]
    self.derivada=registro[5]
    self.setUsuario(registro[6])

  def setUsuario(self, *args):
    pass

  def getEstado(self):
    return [self.output,
            self.time,
            self.senal,
            self.prop,
            self.integral,
            self.derivada,
            self.getUsuario()]

  def getUsuario(self):
    pass

  def setup(self):
    self.enc1.setAnalizador(analizador=self.paraEncoder)
    self.enc2.setAnalizador(analizador=self.paraEncoder)

  def paraEncoder(self, **kwargs):
    if 'time' in kwargs:
      if kwargs['time']>self.time:
        self.analisis(**kwargs)
        self.time=kwargs['time']
    else:
      self.analisis(**kwargs)

  def initUsuario(self, **kwargs):
    pass

  def __call__(self, **kwargs):
    self.senal=clip(self.senal+self.filtro*(self.output-self.senal),
                    maximum=self.maxOutput)
    self.derivada=(self.senal-self.prop)/self.Dt
    self.integral=((self.senal+self.integral)%self.maxOutput)*self.Dt
    self.prop=self.senal
    self.output=self.pid[0]*self.prop
    self.output+=self.pid[1]*self.integral
    self.output+=self.pid[2]*self.derivada
    return self.peso*clip(self.output, maximum=self.maxOutput)

  def analisis(self, **kwargs):
    # El ususario tiene que sobreescribir esta funciÃƒÂ³n
    print "Aca no deberias entrar"
    pass

class SeguirFrec(EncAnalizador):
  '''
  '''
  def initUsuario(self, **kwargs):
    self.flancoAnterior=[0.0, 0.0]
    self.pulsoAnterior=[1, 1]
    self.periodo1=10000000000.0
    self.periodo2=10000000000.0
    self.output=0.0
    self.pulsoReferencia=kwargs['referencia'] if 'referencia' in kwargs else 0

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
    time=kwargs['time'] if 'time' in kwargs else self.time
    ixP=self.pulsoReferencia
    p1 = self.enc1.getPulsos()
    p2 = self.enc2.getPulsos()
    flanco1=p1[ixP]-self.pulsoAnterior[0]
    if flanco1>0:
      self.periodo1=(time-self.flancoAnterior[0])*self.enc1.getResolucion()
      self.flancoAnterior[0]=time
    flanco2=p2[ixP]-self.pulsoAnterior[1]
    if flanco2>0:
      self.periodo2=(time-self.flancoAnterior[1])*self.enc2.getResolucion()
      self.flancoAnterior[1]=time
    self.pulsoAnterior[0]=p1[ixP]
    self.pulsoAnterior[1]=p2[ixP]
    self.senal=self.periodo2-self.periodo1

class PID(EncAnalizador):
  ''' '''
  def initUsuario(self, **kwargs):
    self.flancoAnterior=[.0, .0]
    self.pulsoAnterior=[1, 1]
    self.periodo=[10000.0, 10000.0]
    self.encoders=[self.enc1, self.enc2]

  def setUsuario(self, registro):
    self.flancoAnterior=registro[0]
    self.pulsoAnterior=registro[1]
    self.periodo=registro[2]

  def getUsuario(self):
    return [self.flancoAnterior,
            self.pulsoAnterior,
            self.periodo]

  def analisis(self,**kwargs):
    time=kwargs['time'] if 'time' in kwargs else self.time
    for n, enc in enumerate(self.encoders):
      p=enc.getPulsos()
      flanco=p[0]-self.pulsoAnterior[n]; self.pulsoAnterior[n]=p[0]
      if flanco>0:
        self.periodo[n]=(time-self.flancoAnterior[n])*enc.getResolucion()
        self.flancoAnterior[n]=time
    self.senal=self.periodo[1]-self.periodo[0]

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
    time=kwargs['time'] if 'time' in kwargs else self.time
    Dt=kwargs['Dt'] if 'Dt' in kwargs else 1./self.maxOuput
    elOtro=[1, 0]
    desfase=self.output
    for n, enc in enumerate(self.encoders):
      p=enc.getPulsos()
      flanco=p[2]-self.pulsoAnterior[n]; self.pulsoAnterior[n]=p[2]
      if flanco>0:
        self.periodo[n]=((-1)**n)*(time-self.flancoAnterior[elOtro[n]])
      from math import fabs, copysign
      if fabs(desfase)<fabs(self.periodo[elOtro[n]]):
        desfase=desfase
      else: desfase=self.periodo[elOtro[n]]
      desfase=1./copysign(max(fabs(desfase),Dt),desfase)
    self.output=desfase+self.filtro*(self.output-desfase)

class FaseDetector(EncAnalizador):
  '''
  '''
  def initUsuario(self, **kwargs):
    time=kwargs['time'] if 'time' in kwargs else 0.0
    self.flancoAnterior=[time, time]
    self.pulsoAnterior=[0, 0]
    self.periodo1=1.0
    self.periodo2=1.0
    self.desfase1=0.0
    self.desfase2=0.0
    self.pulsoReferencia=kwargs['referencia'] if 'referencia' in kwargs else 0

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
    time=kwargs['time'] if 'time' in kwargs else self.time
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
    self.senal = self.desfase2-self.desfase1

class XORdetector(EncAnalizador):
    ''' '''
    def initUsuario(self, **kwargs):
      self.pulsoReferencia=kwargs['referencia'] if 'referencia' in kwargs else 0

    def getEstado(self):
      return [self.pulsoReferencia]

    def setEstado(self, registro):
      self.pulsoReferencia=registro[0]

    def analisis(self, **kwargs):
      ixP = self.pulsoReferencia
      p1 = self.enc1.getPulsos()
      p2 = self.enc2.getPulsos()
      self.senal = -1 if p1[ixP]==p2[ixP] else 1

class Senal():
  '''
  Una clase que compone todos los analizadores utilizados
  y devuelve una unica señal para insertar en el motor
  dirigido. Toma de entrada una lista de objetos EncAnalizardor
  '''
  def __init__(self, analizadores, **kwargs):
    self.lista=analizadores
    self.senal=kwargs['senal'] if 'senal' in kwargs else 0.0
    self.delta=0.0
    self.MAX_DELTA=kwargs['MAX_DELTA'] if 'MAX_DELTA' in kwargs else False

  def __call__(self, **kwargs):
    Dt=kwargs['Dt'] if 'Dt' in kwargs else 1.
    delta=0.0
    for am in self.lista:
      delta+=am(**kwargs)
    self.delta=clip(delta, maximum=self.MAX_DELTA/Dt)
    self.senal+=self.delta*Dt
    return self.senal

  def getEstado(self):
    bag = [ am.getEstado() for am in self.lista ]
    return [ bag, self.senal, self.delta ]

  def setEstado(self, registro):
    for am, reg in zip(self.lista, registro[0]):
      am.setEstado(reg)
    self.senal=registro[1]
    self.delta=registro[2]

def clip(o, maximum=False, minimum=False):
  from math import copysign, fabs
  ret=o
  if maximum: ret=copysign(min([fabs(ret), fabs(maximum)]),ret)
  if minimum: ret=copysign(max([fabs(ret), fabs(minimum)]),ret)
  return ret

def guardar(archivo, cosas):
  from pickle import dump
  f=open(archivo, 'wb')
  dump(cosas, f)
  f.close()

def recuperar(archivo):
  from pickle import load
  f=open(archivo, 'rb')
  cosas=load(f)
  f.close()
  return cosas

# Final de las declaraciones de Clases y Funciones
if __name__=='__main__':

  archivo="registro004.pkl" #
  continuar=0
  Dt=.0005  # Se rompe cuando Dt es menor a .0005
  MAXT=10.

  Res1=8
  Res2=16

  senal=1.
  MAX_DELTA=.008

  K1=1.
  PID1=(1.,0.,.000)
  filtro1=0.6
  K2=1.
  PID2=(.2,1.,.1)

  def target(time,t=2.,b=.0,c=.0,T=40.):
    from math import sin
    return t*(1+b*time**2/MAXT**2)+c*sin(time*6.28/T)

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
             #PID=PID1,
             Dt=Dt)

  ana3 = XORdetector(enc1=mot1.getEncoder(),
                    enc2=mot2.getEncoder(),
                    peso=K1,
                    PID=PID1,
                    Dt=Dt)

  ana2 = FaseDetector(enc1=mot1.getEncoder(),
                enc2=mot2.getEncoder(),
                peso=K2,
                PID=PID2,
                Dt=Dt )

  sen=Senal([ana1, ana2],
            MAX_DELTA=MAX_DELTA,
            senal=senal)

  # En la CajaDeZapatos se ponen los objetos cuyos
  # cuyos estados se quieran guardar y recuperar.
  CajaDeZapatos=[mot1, mot2, sen]

  if continuar:
    cosas=recuperar(archivo)
    for donde, que in zip(CajaDeZapatos, cosas[0]):
      donde.setEstado(que)
    time=cosas[1]; MAXT+=time
  else: time=0.0

  X=[]
  reg=[]; deltas=[]; trg=[]
  enc1=[]; enc2=[]

  while time<MAXT :
    time+=Dt; X.append(time)
    kwargs={'time':time, 'Dt':Dt}
    mot1.roll(senal=target(time), **kwargs)
    mot2.roll(senal=sen(**kwargs), **kwargs)
    enc1.append( mot1.getEncoder().getPulsos(shift=.1) )
    enc2.append( mot2.getEncoder().getPulsos() )
    reg.append(sen.senal)
    deltas.append(ana3.senal)
    trg.append(target(time))

  print 'Finales: '
  print 'Eje 1: ', mot1.getEstado()
  print 'Eje 2: ', mot2.getEstado()
  print 'Time : ', time
  print 'Senal: ', sen.senal

  cosas=[]
  for de in CajaDeZapatos:
    cosas.append(de.getEstado())
  cosas=[cosas, time]
  guardar(archivo, cosas)

  import matplotlib.pyplot as plt
  fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)
  A1, B1, Z1 = zip(*enc1)
  A2, B2, Z2 = zip(*enc2)
  ax1.plot(X, A1, X, A2)
  ax2.plot(X, deltas)
  ax3.plot(X, trg, X, reg)
  ax4.plot(X, Z1, X, Z2)

  plt.show()
