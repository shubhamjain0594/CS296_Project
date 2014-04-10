import re
filename="../doc/cs296_report_02.tex"
textfile = open(filename,'r')
filename1="../doc/g02_project_report.html"
textfile1=open(filename1,'w')
textfile1.write('<!DOCtype html>\n<html>' '<link href="index.css" rel="stylesheet" type="text/css" />')
section=False
for line in textfile:
	k = line.strip()
	k = k.replace(	"\\\\", "<br>")
	s="%"
	if s in k:
		k=k[:k.index(s)]
		k="".join(k)
	if re.search(r"^\\subsection\{Profiling", k):
		section=True
	if re.search(r"^\\section", k) and section:
		m=k.partition('{')[-1].rpartition('}')[0]
		l="<h1>" + m + "</h1>\n"
		if re.search(r"^Profil",m):
			break
		textfile1.write(l)
	elif re.search(r"^\\subsection", k) and section:
		m=k.partition('{')[-1].rpartition('}')[0]
		l="<h2>" + k.partition('{')[-1].rpartition('}')[0] + "</h2>\n"
		textfile1.write(l)
	elif re.search(r"^\\title", k) and section:
		m=k.partition('{')[-1].rpartition('}')[0]
		l="<h1>" + k.partition('{')[-1].rpartition('}')[0] + "</h1>\n"
		textfile1.write(l)	
	elif re.search(r"^\\begin\{center}",k) and section:
		try:
			line=next(textfile)
		except StopIteration:
			break
		k=line.strip()
		l=k.partition('{')[-1].rpartition('}')[0]
		image=l
		try:
			line=next(textfile)
		except StopIteration:
			break
		k=line.strip()
		l='<figure>'+'<p align="center"> <img src="../doc/' + image + '.jpg"> </p>'
		textfile1.write(l)
	elif re.search(r"^\\caption",k) and section:
		k=line.strip()
		l=k.partition('{')[-1].rpartition('}')[0]
		caption=l
		k=line.strip()
		l='<figcaption align="center">' + caption +  '</figcaption> </figure>'
		textfile1.write(l)
	elif re.search(r"^\\author", k) and section:
		for line1 in textfile:
			if not "\}" in line1:
				k=k+line1
			else:
				k=k+line1
				break
		l="authors: <br>" + k.partition('{')[-1].rpartition('}')[0] + "\n"
		l = l.replace("\\\\", "<br>")
		s="%"
		if s in l:
			l=l[:l.index(s)]
			l="".join(l)
		textfile1.write(l)
	elif (not re.search(r"^\%", k)) and section:
		if not (re.search(r"^\\usepackage", k) or re.search(r"^\\date",k) or re.search(r"^\\document", k) or re.search(r"^\\biblio", k) or re.search(r"^\\begin", k) or re.search(r"^\\end",k) or re.search(r"^\\maketitle", k)):
			l1=k+"\n"
			textfile1.write(l1)
textfile1.write('</html>')
