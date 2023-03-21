function shp=shape(nod)
    %nod=6;dis=1;
    distance=1;
    sides=nod;
    points=zeros(sides,2);

    for i=1:1:sides-1
        tp=[points(i,1) points(i,2)];
        cp=[points(i,1)+distance points(i,2)]-tp;
        angle=(i-1)*(2*pi/sides);
        np=(cp*[cos(angle) -sin(angle); sin(angle) cos(angle)])+tp;
        points(i+1,:)=np;
    end

    if sides>2
        A=[points(1,1) points(1,2)];
        B=[points(2,1) points(2,2)];
        C=[points(3,1) points(3,2)];
        tempd=(C(1)-B(1))*(A(2)-C(2))+(B(2)-C(2))*(A(1)-C(1));
        ra=(C(1)^2+C(2)^2-B(1)^2-B(2)^2)/2;
        rb=(A(1)^2+A(2)^2-C(1)^2-C(2)^2)/2;
        rx=(ra*(A(2)-C(2))-rb*(C(2)-B(2)))/tempd;
        ry=(rb*(C(1)-B(1))-ra*(A(1)-C(1)))/tempd;
        coc=[rx,ry];
        points=points-coc;
    else
        points=points-[0.5*distance,0];
    end
    shp=points;
end