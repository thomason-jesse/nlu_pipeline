#program base.
person(alice).
person(frannie).
person(bob).
person(carol).
person(dave).
person(george).
person(eve).
person(mallory).
person(peggy).
person(walter).

group(bwi).

ingroup(alice,bwi).
ingroup(frannie,bwi).
ingroup(bob,bwi).
ingroup(carol,bwi).
ingroup(dave,bwi).
ingroup(george,bwi).
ingroup(eve,bwi).
ingroup(mallory,bwi).
ingroup(peggy,bwi).
ingroup(walter,bwi).

hasoffice(mallory,l3_508). 
hasoffice(walter,l3_512).
hasoffice(frannie,l3_510). 
hasoffice(bob,l3_402). 
hasoffice(eve,l3_418).
hasoffice(alice,l3_502).

%students in the lab
hasoffice(carol,l3_414b).
hasoffice(dave,l3_414b).
hasoffice(peggy,l3_414b).

canbeinroom(P,R) :- hasoffice(P,R), person(P), room(R).
canbeinroom(P,l3_414b) :- ingroup(P,bwi).

canknow(P1,P2) :- ingroup(P1,G), ingroup(P2,G), P1 != P2, group(G).

canknow(P2,P1) :- canknow(P1,P2).

meeting(M,G,R) :- meeting(M,G,R). %here for when not using meetings

%#hide person/1.
%#hide hasoffice/2.
%#hide canbeinroom/2.
%#hide canknow/2.
%#hide group/1.
%#hide ingroup/2.
%#hide meeting/3.
