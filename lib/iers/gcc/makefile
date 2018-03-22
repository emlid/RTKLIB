# makefile for iers.a

SRC      = ../src

CCFLAGS = -O3 -ffixed-line-length-132

OBS = cal2jd.o dat.o dehanttideinel.o gmf.o gpt.o norm8.o sprod.o st1idiu.o \
      st1isem.o st1l1.o step2diu.o step2lon.o vmf1.o vmf1_ht.o zero_vec8.o

iers.a     : $(OBS)
	ar ru iers.a $(OBS)
cal2jd.o   : $(SRC)/cal2jd.f
	$(CC) -c $(CCFLAGS) $(SRC)/cal2jd.f
dat.o      : $(SRC)/dat.f
	$(CC) -c $(CCFLAGS) $(SRC)/dat.f
dehanttideinel.o : $(SRC)/dehanttideinel.f
	$(CC) -c $(CCFLAGS) $(SRC)/dehanttideinel.f
gmf.o      : $(SRC)/gmf.f
	$(CC) -c $(CCFLAGS) $(SRC)/gmf.f
gpt.o      : $(SRC)/gpt.f
	$(CC) -c $(CCFLAGS) $(SRC)/gpt.f
norm8.o    : $(SRC)/norm8.f
	$(CC) -c $(CCFLAGS) $(SRC)/norm8.f
sprod.o    : $(SRC)/sprod.f
	$(CC) -c $(CCFLAGS) $(SRC)/sprod.f
st1idiu.o  : $(SRC)/st1idiu.f
	$(CC) -c $(CCFLAGS) $(SRC)/st1idiu.f
st1isem.o  : $(SRC)/st1isem.f
	$(CC) -c $(CCFLAGS) $(SRC)/st1isem.f
st1l1.o    : $(SRC)/st1l1.f
	$(CC) -c $(CCFLAGS) $(SRC)/st1l1.f
step2diu.o : $(SRC)/step2diu.f
	$(CC) -c $(CCFLAGS) $(SRC)/step2diu.f
step2lon.o : $(SRC)/step2lon.f
	$(CC) -c $(CCFLAGS) $(SRC)/step2lon.f
vmf1.o     : $(SRC)/vmf1.f
	$(CC) -c $(CCFLAGS) $(SRC)/vmf1.f
vmf1_ht.o  : $(SRC)/vmf1_ht.f
	$(CC) -c $(CCFLAGS) $(SRC)/vmf1_ht.f
zero_vec8.o: $(SRC)/zero_vec8.f
	$(CC) -c $(CCFLAGS) $(SRC)/zero_vec8.f

clean :
	rm -f *.o *.a

