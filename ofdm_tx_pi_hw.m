function ofdm_tx_pi_hw()
%#codegen
coder.cinclude('udp_linux.h');
coder.updateBuildInfo('addSourceFiles', 'udp_linux.c');

% setare parametri ofdm, frecvențe nule și piloți
nfft = 64; cpLen = 16; nSym = 20; No_frames = 50;
nullIdx  = [1:6 33 60:64]'; pilotIdx = [12 26 40 54]'; pilots = repmat([1;1;1;-1],1,nSym+1);
M_head = 2; M = 64; 
numDataCarrs = nfft - length(nullIdx) - length(pilotIdx);

tic; 

% preambulul pentru sincronizarea receptorului
qamSynch1=[0,0,0,0,0,0,0,1.4142,0,-1.4142,0,1.4142,0,-1.4142,0,-1.4142,0,-1.4142,0,1.4142,0,-1.4142,0,1.4142,0,-1.4142,0,-1.4142,0,-1.4142,0,-1.4142,0,1.4142,0,-1.4142,0,1.4142,0,1.4142,0,1.4142,0,-1.4142,0,1.4142,0,1.4142,0,1.4142,0,-1.4142,0,1.4142,0,1.4142,0,1.4142,0,0,0,0,0,0];
qamSynch2=[0,0,0,0,0,0,-1,-1,-1,-1,1,1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,-1,-1,1,-1,0,1,-1,1,1,1,-1,1,1,1,-1,1,1,1,1,-1,1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,0,0,0,0,0];
qamSynch1(end+1:length(qamSynch2))=0; qamSinch = [qamSynch1', qamSynch2'];

% trecere prin modulatorul ofdm
outSigSinc = ofdmmod(qamSinch, nfft, cpLen);

% generare șir determinist pentru header
inHead = mod(0:numDataCarrs-1, M_head)';
qamHead = qammod(inHead, M_head, 'UnitAveragePower', true);

% generare bloc date utile
inData = mod(0:(numDataCarrs*nSym)-1, M)';
inData = reshape(inData, numDataCarrs, nSym);
qamData = qammod(inData, M, 'UnitAveragePower', true);

% concatenare header si date utile în frecvență
qamSym = [qamHead, qamData];

% trecere prin ifft pentru adăugarea prefixului ciclic
outSigHData = ofdmmod(qamSym, nfft, cpLen, nullIdx, pilotIdx, pilots);
% lipire preambul cu fluxul de date in timp
outSig = [outSigSinc; outSigHData]; 

% copiere repetitivă a aceluiași cadru pentru anduranța simulării
outSigLong = repmat(outSig, No_frames, 1);

timp_modulare = toc; 
fprintf(1, '[TX T8] Modulare completa in %.4f secunde.\n', timp_modulare);

% simulare canal fizic cu ecou
delay_esantioane = 15; 
h_canal = complex(zeros(20, 1)); 
h_canal(delay_esantioane + 1) = 1;        
h_canal(end) = 0.4 - 0.2i;                
% filtrare semnal transmis prin modelul canalului
outSigLong_ch = filter(h_canal, 1, outSigLong(:)); 

% adaugare zgomot termic simulând aerul ca mediu de propagare
outSigLong_ch = awgn(outSigLong_ch, 30, 'measured'); 

% zona de siguranță la finalul pachetului
linieste_finala = complex(zeros(100, 1)); 
outSigLong_ch = [outSigLong_ch; linieste_finala]; 

% separare eșantioane în vector tip bytes pentru socketul de c
payload = zeros(2*numel(outSigLong_ch), 1, 'single');
payload(1:2:end) = real(outSigLong_ch); 
payload(2:2:end) = imag(outSigLong_ch);
payload_bytes = typecast(payload,'uint8');

% împărțire mesaj în pachete digerabile de udp
chunkSize = 1024; numBytes = numel(payload_bytes); numChunks = ceil(numBytes/chunkSize);

sock = int32(0); sock = coder.ceval('udp_init_tx');
fprintf(1, '[TX T8] Trimit %d pachete (Port 5005)...\n', int32(numChunks));
fixed_chunk = zeros(chunkSize, 1, 'uint8');

% loop trimitere pachete succesive
for k = 1:numChunks
    startIdx = (k-1)*chunkSize + 1; endIdx = min(k*chunkSize, numBytes);
    actual_len = endIdx - startIdx + 1;
    fixed_chunk(1:actual_len) = payload_bytes(startIdx:endIdx);
    coder.ceval('udp_send', sock, coder.rref(fixed_chunk), int32(actual_len));
    coder.ceval('udp_delay', int32(10)); 
end

coder.ceval('udp_close', sock);
fprintf(1, '[TX T8] Gata transmisia!\n');
end