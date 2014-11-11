### Stuff for user programming ###
dbLoadRecords("$(CALC)/calcApp/Db/userCalcs10.db","P=xxx:")
dbLoadRecords("$(CALC)/calcApp/Db/userCalcOuts10.db","P=xxx:")
#dbLoadRecords("$(CALC)/calcApp/Db/userCalcOuts10more.db","P=xxx:,N1=11,N2=12,N3=13,N4=14,N5=15,N6=16,N7=17,N8=18,N9=19,N10=20")
dbLoadRecords("$(CALC)/calcApp/Db/userStringCalcs10.db","P=xxx:")
dbLoadRecords("$(CALC)/calcApp/Db/userArrayCalcs10.db","P=xxx:,N=8000")
dbLoadRecords("$(CALC)/calcApp/Db/userTransforms10.db","P=xxx:")
dbLoadRecords("$(CALC)/calcApp/Db/userAve10.db","P=xxx:")
# string sequence (sseq) records
dbLoadRecords("$(CALC)/calcApp/Db/userStringSeqs10.db","P=xxx:")
# editSseq - edit any sseq or seq record
dbLoadRecords("$(CALC)/calcApp/Db/editSseq.db", "P=xxx:,Q=ES:")
doAfterIocInit("seq &editSseq, 'P=xxx:,Q=ES:'")

# interpolation
dbLoadRecords("$(CALC)/calcApp/Db/interp.db", "P=xxx:,N=2000")
dbLoadRecords("$(CALC)/calcApp/Db/interpNew.db", "P=xxx:,Q=1,N=2000")
# busy record
dbLoadRecords("$(BUSY)/busyApp/Db/busyRecord.db", "P=xxx:,R=mybusy1")
dbLoadRecords("$(BUSY)/busyApp/Db/busyRecord.db", "P=xxx:,R=mybusy2")