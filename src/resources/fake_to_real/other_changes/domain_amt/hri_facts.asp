#program base.
person(stacy).
person(jesse).
person(shiqi).
person(jivko).
person(aishwarya).
person(rodolfo).
person(peter).
person(dana).
person(ray).
person(scott).

item(hamburger).
item(coffee).
item(juice).
item(muffin).
item(chips).

group(bwi).

ingroup(jesse,bwi).
ingroup(shiqi,bwi).
ingroup(jivko,bwi).
ingroup(aishwarya,bwi).
ingroup(rodolfo,bwi).
ingroup(peter,bwi).
ingroup(dana,bwi).
ingroup(ray,bwi).
ingroup(scott,bwi).

hasoffice(stacy,l3_502).
hasoffice(shiqi,l3_420).
hasoffice(jivko,l3_432).
hasoffice(peter,l3_508). 
hasoffice(dana,l3_510).
hasoffice(ray,l3_512).
hasoffice(scott,l3_404).

hasoffice(jesse,l3_414b).
hasoffice(aishwarya,l3_414b).
hasoffice(rodolfo,l3_414b).

canbeinroom(P,R) :- hasoffice(P,R), person(P), room(R).
canbeinroom(P,l3_414b) :- ingroup(P,bwi).

canknow(P1,P2) :- ingroup(P1,G), ingroup(P2,G), P1 != P2, group(G).

canknow(P2,P1) :- canknow(P1,P2).

meeting(M,G,R) :- meeting(M,G,R). %here for when not using meetings

%#hide item/1.
%#hide person/1.
%#hide hasoffice/2.
%#hide canbeinroom/2.
%#hide canknow/2.
%#hide group/1.
%#hide ingroup/2.
%#hide meeting/3.
